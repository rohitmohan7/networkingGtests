//#include "layer4.h"
#include "layer3.h"
#include "allocator.h"
#include "layer2.h"
#include "network.h"
#include "pit.h"

#define L2_FRAME_SIZE (RS485_FRAME_SIZE - sizeof(L2Hdr))
#define L3_FRAME_SIZE (L2_FRAME_SIZE - sizeof(L3Hdr))
#define L4_FRAME_SIZE (L3_FRAME_SIZE - sizeof(L4Hdr))

#define L4_MAX_RETRY 3
#define L4_RETRY_TIMER 100 // 100 ms

stream_t streams[MAX_POS][MAX_PRIORITY];

TxOrderType txOrder = 0;

void l4Init() {
	txOrder = 0;
	for (int prio = 0; prio < MAX_PRIORITY; prio++) {
		for (int pos = 0; pos < MAX_POS; pos++) {
			stream_t* s = &streams[pos][prio];
			s->txOrder = 0;
			memset(&s->txMsgHdr, 0x0, sizeof(L4Hdr));
			s->head_page = INVALID_PAGE;
			s->tail_page = INVALID_PAGE;
			s->head_off = 0;
			s->tail_used = 0;
			s->retryCnt = 0;
			s->retryTmr = 0;
			s->txFrameCnt = 0;
		}
	}
}

uint8_t getL4PktHd(L4Pkt* l4Pkt, uint8_t prio, uint8_t* offset) {
	stream_t* s = &streams[l4Pkt->pos][prio];
	uint8_t currHd = s->head_page;
	uint8_t currOfst = s->head_off;

	// advance by frame count
	uint8_t txBytes = (s->txFrameCnt) * L4_FRAME_SIZE;

	while (txBytes) {
		uint8_t pgBytes = (UNIT - currOfst);

		uint8_t pgTxBytes = min(txBytes, pgBytes);

		if (txBytes > pgTxBytes) { // advance to next page
			currHd = g_next[s->head_page];
			currOfst = 0;
		} else {
			// just advance offset
			currOfst = txBytes;
		}

		txBytes -= pgTxBytes;
	}

	*offset = currOfst;
	s->txMsgHdr.msgFlgs = l4Pkt->hdr.msgLen
		? (s->txMsgHdr.msgFlgs | L4_MSG_FLAG_STREAM_PENDING)
		: (s->txMsgHdr.msgFlgs & ~L4_MSG_FLAG_STREAM_PENDING);
	return currHd;
}

void setL4Hdr(L4Pkt* l4Pkt, uint8_t prio) {
	L4Hdr* l4PktHdr = &l4Pkt->hdr;
	L4Hdr* txHdr = &streams[l4Pkt->pos][prio].txMsgHdr;
	l4PktHdr->msgNo = txHdr->msgNo;
	l4PktHdr->msgFlgs = (txHdr->msgFlgs & ~(L4_MSG_FLAG_STREAM_PENDING | L4_MSG_FLAG_PENDING_ACK)); // unset internal flags
	l4PktHdr->msgLen = (txHdr->msgLen > L4_FRAME_SIZE) ? (txHdr->msgLen - L4_FRAME_SIZE) : 0; // remaining msg len
}

void getL4PktFrag(L4Pkt* l4Pkt, uint8_t** ptr, uint8_t* len, uint8_t* txHd, uint8_t* txHdOfst, uint8_t txLen, uint8_t prio) {
	const uint16_t base = pageOff(*txHd) + (uint16_t)(*txHdOfst);
	// start with first page
	//*ptr = &g_pool[base] + (*txHdOfst);
	//*len = (UNIT - (*txHdOfst));
	*ptr = &g_pool[base];

	const stream_t* s = &streams[l4Pkt->pos][prio];

	// gate for msglen
	uint16_t ofst = pageOff(s->head_page) + (uint16_t)(s->head_off);
	uint16_t prioTxCnt = base - ofst;
	
	// cap tx len to min of msg len/Frame size
	*len = min((UNIT - (*txHdOfst)), 
			min((s->txMsgHdr.msgLen - prioTxCnt), (L4_FRAME_SIZE - prioTxCnt)));
#if 0
	if ((*txHd) == s->tail_page) {
		*len -= (UNIT - s->tail_used);

			//if (txRxFifoLen >= *len) { // all data will be sent in this single Tx
			//	return true;
			//}
		*txHdOfst += txLen;
	}
	else
#endif
	if (*len <= txLen) { // we are at the end of current page
		*txHd = g_next[(*txHd)];
		*txHdOfst = 0;
	}
	else {
		*len = min(txLen, *len);
		*txHdOfst += *len;
	}

	uint16_t newTxLen = *len + prioTxCnt;

	if ((newTxLen >= s->txMsgHdr.msgLen) ||
		newTxLen >= L4_FRAME_SIZE) { // end tx
		*txHd = INVALID_PAGE;
	}
}

static inline void readFromPgs(stream_t* s, uint8_t * val, uint8_t size) {
	for (int i = 0; i < size; i++) {
		if (s->head_page == INVALID_PAGE || 
			(s->head_page == s->tail_page && s->head_off == s->tail_used)) { // TODO deterministic fail
			memset(val, 0, size);
			return;
		}

		uint16_t base = pageOff(s->head_page) + (s->head_off);
		val[i] = (g_pool[base]) << (8 * i);
		s->head_off++;
		if (s->head_off == UNIT) {
			s->head_page = g_next[s->head_page];
			s->head_off = 0;
		}
	}
}

void freeStream(stream_t * s) {
	while (s->head_page != INVALID_PAGE) {
		uint8_t currPage = s->head_page;
		s->head_page = g_next[currPage];
		page_free(currPage);
	}
	s->tail_page = s->head_page;
}

static inline bool clearMsgFrame(L4Pkt* l4Pkt, uint8_t prio, bool allFrames) {
	stream_t* s = &streams[l4Pkt->pos][prio];
	L4Hdr* txHdr = &s->txMsgHdr;
	L4Hdr* hdr = &l4Pkt->hdr;
	// free till Frame length
	uint8_t len = (!allFrames && txHdr->msgLen > L4_FRAME_SIZE) ? L4_FRAME_SIZE: 
		(txHdr->msgLen + (allFrames? hdr->msgLen:0));
	while (len > 0) {

		uint8_t pageLen = UNIT - s->head_off;

		if (pageLen > len) { // page has another frame just advance head
			s->head_off += len;
		}
		else {
			uint8_t currPage = s->head_page;
			s->head_page = g_next[s->head_page];
			page_free(currPage);
			s->head_off = 0;
		}

		len -= (pageLen > len) ? len : pageLen;
	}

	// advance stream
	
	
	if (s->head_page != INVALID_PAGE) {

		if (allFrames || hdr->msgLen == 0) {
			txHdr->msgNo++; // increment seq number
			readFromPgs(s, (uint8_t*)&txHdr->msgLen, sizeof(txHdr->msgLen));

			if (txHdr->msgLen == 0) { //  no more message pending free stream
				freeStream(s);
				return true;
			}
			else {
				readFromPgs(s, (uint8_t*)&s->txOrder, sizeof(TxOrderType));
				readFromPgs(s, (uint8_t*)&s->txMsgHdr.msgFlgs, sizeof(s->txMsgHdr.msgFlgs));
				s->txFrameCnt = 0;
				s->retryTmr = 0;
				s->retryCnt = 0;
			}
		}
		else {
			txHdr->msgLen = hdr->msgLen;
		}
		return false;
	}
	return true;
}

bool l4lstMsgFrm(uint16_t pos, uint8_t prio) {
	stream_t* ps = &streams[pos][prio];
	return (ps->txMsgHdr.msgLen < L4_FRAME_SIZE) || (ps->txMsgHdr.msgLen - (ps->txFrameCnt * L4_FRAME_SIZE)) < L4_FRAME_SIZE;
}

void l4TxCmplt(L4Pkt* l4Pkt, uint8_t prio) {
	 if (!(l4Pkt->hdr.msgFlgs & L4_MSG_FLAG_REQ_ACK)) { // this msg does not require an ack, frame can be cleared from page buffer
		 clearMsgFrame(l4Pkt, prio, false);
	 } else {
		 stream_t* s = &streams[l4Pkt->pos][prio];
		 s->txFrameCnt++;
		if (l4lstMsgFrm(l4Pkt->pos, prio)) {
			s->txMsgHdr.msgFlgs |= L4_MSG_FLAG_PENDING_ACK;
			//set the timer
			s->retryTmr = pitGetCurrMS();
		}
	 }
}

bool l4StrmEmptyAftFrme(uint16_t pos, uint8_t prio) {
	stream_t* s = &streams[pos][prio];

	if ((s->txMsgHdr.msgFlgs & L4_MSG_FLAG_REQ_ACK) && l4lstMsgFrm(pos, prio)) {
		// after last frame this stream will wait for ack before moving to next message, consider empty after frame...
		return true;
	}

	// check size remaining in page buff
	const uint16_t base = pageOff(s->head_page) + (uint16_t)(s->head_off);
	const uint16_t end = pageOff(s->tail_page) + (uint16_t)(s->tail_used);

	return (end - base) <= L4_FRAME_SIZE;
}

bool l4StrmEmpty(uint16_t pos, uint8_t prio) {
	stream_t* ps = &streams[pos][prio];
	if (ps->head_page == INVALID_PAGE) {
		return true;
	}
	return false;
}

bool l4StrmPnding(uint8_t pos, uint8_t prio) {
	stream_t* ps = &streams[pos][prio];
	if (ps->txMsgHdr.msgFlgs & L4_MSG_FLAG_STREAM_PENDING) {
		return true;
	}
	return false;
}

bool getL4Pkt(L4Pkt* l4Pkt, uint16_t pos, uint8_t prio, uint8_t pktPrio) {
	TxOrderType currTxOrder = (l4Pkt->pos < MAX_POS) ? streams[l4Pkt->pos][pktPrio].txOrder : ~0U;

	stream_t* ps = &streams[pos][prio];
	
	if (ps->head_page != INVALID_PAGE &&
		ps->txOrder <= currTxOrder) { // nothing to send

		if ((ps->txMsgHdr.msgFlgs & L4_MSG_FLAG_PENDING_ACK))
		{ // check if this message has timed out
			if (pitTimespanExceeded(ps->retryTmr, pitGetCurrMS(), L4_RETRY_TIMER))
			{
				if (ps->retryCnt >= MAX_L4_RETRY)
				{
					// drop move and move to next msg
					const bool strmEmpty = clearMsgFrame(l4Pkt, prio, true);
					const bool txOrdrPrempt = ((l4Pkt->pos != pos) || (pktPrio != prio)) && 
						(ps->txOrder > currTxOrder);

					if (strmEmpty || txOrdrPrempt) {
						return false;
					}
				} else {
					ps->txMsgHdr.msgFlgs &= ~L4_MSG_FLAG_PENDING_ACK;
					ps->retryCnt++;
					ps->txFrameCnt = 0;
				}
			} else {
				return false;
			}
		}

		l4Pkt->pos = pos;
		return true;
	}

	return false;
}

#if 0
void l4Tick(uint8_t ms) {
	for (uint16_t pos = 0; pos < MAX_POS; pos++) {
		for (uint8_t prio = 0; prio < MAX_PRIORITY; prio++) {
			stream_t* s = &streams[pos][prio];
			if ((s->txMsgHdr.msgFlgs & L4_MSG_FLAG_PENDING_ACK)) {
				s->retryTmr = (((uint16_t)s->retryTmr) + (uint16_t)ms) > 0xFF ? 0xFF : s->retryTmr + ms;

				if (s->retryTmr > L4_RETRY_TIMEOUT) {
					if (s->retryCnt >= MAX_L4_RETRY) {
						// drop move and move to next msg
					}
					else {
						s->retryCnt++;
						s->txMsgHdr.msgFlgs &= ~L4_MSG_FLAG_PENDING_ACK;
						s->retryTmr = 0;
					}
				}
			}
		}
	}
}
// directly enqueue next prio msg in ACK
bool l4Ack(L4Pkt* l4Pkt, uint8_t prio, uint16_t dstAddr) {


	for (int pos = 0; pos < MAX_POS; pos++) {
		stream_t* s = &streams[pos];
		if (s->dst != dstAddr) {
			continue;
		}

		prio_stream_t* ps = &s->prio[prio];

		// free till tail
		while (ps->head_page != l4Pkt->tail_page) {
			uint8_t currPage = ps->head_page;
			ps->head_page = g_next[ps->head_page];
			page_free(currPage);
		}

		// free from allocator
		if (l4Pkt->tail_used == UNIT) { // full tail used 
			page_free(ps->head_page);
			ps->head_page = g_next[l4Pkt->tail_page];
			ps->head_off = 0;
		}
		else {
			//ps->head_page = l4Pkt->tail_page;
			ps->head_off = l4Pkt->tail_used;
		}

		L4Hdr* hdr = &l4Pkt->hdr;
		L4Hdr* txHdr = &ps->txMsgHdr;
		if (ps->head_page != INVALID_PAGE) {

			if (hdr->msgLen == 0) {
				txHdr->msgNo++; // increment seq number

				for (int i = 0; i < sizeof(txHdr->msgLen); i++) {
					uint16_t base = pageOff(ps->head_page) + (ps->head_off);
					txHdr->msgLen |= (g_pool[base]) << (8*i);
					ps->head_off++;
					if (ps->head_off == UNIT) {
						ps->head_page = g_next[ps->head_page];
						ps->head_off = 0;
					}
				}

				if (txHdr->msgLen == 0) { //  no more message pending
					ps->head_page = INVALID_PAGE;
					ps->tail_page = INVALID_PAGE;
				}
				else { //  get tx order
					for (int i = 0; i < sizeof(ps->txOrder); i++) {
						uint16_t base = pageOff(ps->head_page) + (ps->head_off);
						ps->txOrder |= (g_pool[base]) << (8 * i);
						ps->head_off++;
						if (ps->head_off == UNIT) {
							ps->head_page = g_next[ps->head_page];
							ps->head_off = 0;
						}
					}
				}
				return true;
			}
			else {
				txHdr->msgLen = hdr->msgLen;
			}
		}
		
		queueMsg(l4Pkt, ps);
		return false;
		//break;
	}
}
	
void queueMsg(L4Pkt* l4Pkt, stream_t* ps) {
	// set hdr
	L4Hdr* l4PktHdr = &l4Pkt->hdr;
	L4Hdr* txHdr = &ps->txMsgHdr;
	memcpy(l4PktHdr, txHdr, (sizeof(L4Hdr) - sizeof(MsgLenType)));
	l4PktHdr->msgLen = (txHdr->msgLen > L4_FRAME_SIZE) ? (txHdr->msgLen - L4_FRAME_SIZE) : 0; // remaining msg len
	//l4Pkt->s = ps;
#if 0
	// feed packet
	l4Pkt->head_page = ps->head_page;
	l4Pkt->head_off = ps->head_off;

	// debug
	uint16_t base = pageOff(l4Pkt->head_page) + (uint16_t)l4Pkt->head_off;

	// debug
	uint8_t currHd = l4Pkt->head_page;
	uint8_t len = min(L4_FRAME_SIZE, txHdr->msgLen); // write until min msg len

	while (len > 0) {
		if (/*currHd == ps->tail_page ||*/
			len < (UNIT)) { // reached end
			l4Pkt->tail_page = currHd;

			if (currHd == ps->head_page) {
				l4Pkt->tail_used = ps->head_off + len;
			}
			else {
				l4Pkt->tail_used = /*currHd == ps->tail_page ? min(len, ps->tail_used) :*/ len;
			}
			base = pageOff(l4Pkt->tail_page) + (uint16_t)l4Pkt->tail_used;
			break;
		}

		if (currHd == l4Pkt->head_page) {
			len -= (UNIT - l4Pkt->head_off);
		}
		else {
			len -= UNIT;
		}

		currHd = g_next[currHd];

	}
#endif
}
#endif
