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
	
	// init streams
	txOrder = 0;
	for (int prio = 0; prio < MAX_PRIORITY; prio++) {
		for (int pos = 0; pos < MAX_POS; pos++) {
			stream_t* s = &streams[pos][prio];
			s->txOrder = 0;
			memset(&s->txMsgHdr, 0x0, sizeof(L4Hdr));
			memset(&s->rxMsgHdr, 0x0, sizeof(L4Hdr));
			s->head_page = INVALID_PAGE;
			s->tail_page = INVALID_PAGE;
			s->head_off = 0;
			s->tail_used = 0;
			
			/* rx */
			pgPtrInit(&s->rxPgPtr);

			s->retryCnt = 0;
			s->retryTmr = 0;
			s->txFrameCnt = 0;
		}
	}
}

bool getL4PktHd(L4Pkt* l4Pkt, uint8_t prio, uint8_t* hd, uint8_t* offset) {
	stream_t* s = &streams[l4Pkt->pos][prio];

	if (s->txMsgHdr.msgFlgs & L4_MSG_FLAG_TYPE_ACK) // this is an ack message only has hdr
	{
		return true;
	}

	uint8_t currHd = s->head_page;
	uint8_t currOfst = s->head_off;

	// advance by frame count
	uint8_t txBytes = (s->txFrameCnt) * L4_FRAME_SIZE;

	while (txBytes) {
		uint8_t pgBytes = (UNIT - currOfst);

		uint8_t pgTxBytes = min(txBytes, pgBytes);

		if (txBytes > pgTxBytes) { // advance to next page
			currHd = g_next[currHd];
			currOfst = 0;
		} else {
			// just advance offset
			currOfst = txBytes;
		}

		txBytes -= pgTxBytes;
	}

	*offset = currOfst;
	*hd = currHd;

	s->txMsgHdr.msgFlgs = l4Pkt->hdr.msgLen
							  ? (s->txMsgHdr.msgFlgs | L4_MSG_FLAG_STREAM_PENDING)
							  : (s->txMsgHdr.msgFlgs & ~L4_MSG_FLAG_STREAM_PENDING);
	return false;
}

void setL4Hdr(L4Pkt* l4Pkt, uint8_t prio) {
	L4Hdr* l4PktHdr = &l4Pkt->hdr;
	stream_t* ps = &streams[l4Pkt->pos][prio];
	L4Hdr* txHdr = &ps->txMsgHdr;
	l4PktHdr->msgNo = txHdr->msgNo;
	l4PktHdr->msgFlgs = (txHdr->msgFlgs & ~(L4_MSG_FLAG_STREAM_PENDING | L4_MSG_FLAG_PENDING_ACK)); // unset internal flag
	l4PktHdr->msgLen = ((txHdr->msgLen - (ps->txFrameCnt * L4_FRAME_SIZE)) > L4_FRAME_SIZE) ? 
		((txHdr->msgLen - (ps->txFrameCnt * L4_FRAME_SIZE)) - L4_FRAME_SIZE) : 0; // remaining msg len
}

uint8_t getL4PktFrag(L4Pkt* l4Pkt, uint8_t** ptr, uint8_t idx, uint8_t* txHd, uint8_t* txHdOfst, uint8_t txLen, uint8_t prio) {
	const uint16_t base = pageOff(*txHd) + (uint16_t)(*txHdOfst);

	*ptr = &g_pool[base];

	const stream_t* s = &streams[l4Pkt->pos][prio];

	// gate for msglen
	uint16_t prioTxCnt = (s->txFrameCnt * L4_FRAME_SIZE) + idx;
	
	// cap tx len to min of msg len/Frame size
	uint8_t len = min((UNIT - (*txHdOfst)), 
			min((s->txMsgHdr.msgLen - prioTxCnt), (L4_FRAME_SIZE - (prioTxCnt % L4_FRAME_SIZE))));

	if (len <= txLen) { // we are at the end of current page
		*txHd = g_next[(*txHd)];
		*txHdOfst = 0;
	}
	else {
		len = min(txLen, len);
		*txHdOfst += len;
	}

	uint16_t newTxLen = len + prioTxCnt;

	if ((newTxLen >= s->txMsgHdr.msgLen) ||
		newTxLen >= L4_FRAME_SIZE) { // end tx
		*txHd = INVALID_PAGE;
	}
	
	return len;
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
			uint8_t currPage = s->head_page;
			s->head_page = g_next[s->head_page];
			s->head_off = 0;
			page_free(currPage);
		}
	}
}

static inline void freeStream(stream_t *s)
{
	while (s->head_page != INVALID_PAGE) {
		uint8_t currPage = s->head_page;
		s->head_page = g_next[currPage];
		page_free(currPage);
	}
	s->tail_page = s->head_page;
	s->txMsgHdr.msgFlgs = 0;
}

static inline void freeRxStream(stream_t *s)
{
	freePgPtr(&s->rxPgPtr);
	s->rxMsgHdr.msgFlgs = 0;
}

static inline bool clearMsgFrame(L4Pkt *l4Pkt, stream_t *s, bool allFrames)
{
	L4Hdr* txHdr = &s->txMsgHdr;
	L4Hdr *hdr = l4Pkt? &l4Pkt->hdr: NULL;
	// free till Frame length
	uint8_t len = (!allFrames && txHdr->msgLen > L4_FRAME_SIZE) ? L4_FRAME_SIZE: 
		(txHdr->msgLen + ((allFrames && hdr)? hdr->msgLen:0));
	while (len > 0) {

		uint8_t pageLen = UNIT - s->head_off;

		if (pageLen > len) { // page has another msg just advance head
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

		if (allFrames || !hdr || hdr->msgLen == 0) {
			txHdr->msgNo++; // increment seq number
			readFromPgs(s, (uint8_t*)&txHdr->msgLen, sizeof(txHdr->msgLen));
			readFromPgs(s, (uint8_t*)&s->txOrder, sizeof(TxOrderType));
			readFromPgs(s, (uint8_t*)&s->txMsgHdr.msgFlgs, sizeof(s->txMsgHdr.msgFlgs));

			if (txHdr->msgLen == 0 && 
				s->txMsgHdr.msgFlgs == 0) { //  no more message pending free stream
				freeStream(s);
				return true;
			}
			else {
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
	stream_t *s = &streams[l4Pkt->pos][prio];
	if (!(l4Pkt->hdr.msgFlgs & L4_MSG_FLAG_REQ_ACK)) { 
		// this msg does not require an ack, frame can be cleared from page buffer
		clearMsgFrame(l4Pkt, s, false);
	} else {
		 
		 if (l4lstMsgFrm(l4Pkt->pos, prio)) {
			 s->txMsgHdr.msgFlgs |= L4_MSG_FLAG_PENDING_ACK;
			 //set the timer
			 s->retryTmr = pitGetCurrMS();
		 }
		 s->txFrameCnt++;
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

	if ((ps->head_page != INVALID_PAGE || ps->txMsgHdr.msgFlgs & L4_MSG_FLAG_TYPE_ACK) &&
		ps->txOrder <= currTxOrder)
	{ // nothing to send

		if ((ps->txMsgHdr.msgFlgs & L4_MSG_FLAG_PENDING_ACK))
		{ // check if this message has timed out
			if (pitTimespanExceeded(ps->retryTmr, pitGetCurrMS(), L4_RETRY_TIMER))
			{
				if (ps->retryCnt >= MAX_L4_RETRY)
				{
					// drop move and move to next msg
					const bool strmEmpty = clearMsgFrame(l4Pkt, ps, true);
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

void writeValToPage(stream_t *s, uint8_t *val, uint8_t len)
{
	for (int size = 0; size < len; size++)
	{
		if (s->tail_used == UNIT)
		{
			uint8_t p = page_alloc();
			g_next[s->tail_page] = p;
			s->tail_page = page_alloc();
			g_pool[pageOff(s->tail_page)] = val[size];
			s->tail_used = 1U;
		}
		else
		{
			const uint32_t base = pageOff(s->tail_page) + (uint32_t)s->tail_used;
			g_pool[base] = val[size];
			s->tail_used++;
		}
	}
}

void l4SndAck(stream_t *const s)
{
	MsgLenType len = 0;
	uint8_t msgFlgs = L4_MSG_FLAG_TYPE_ACK;
	if (s->tail_page != INVALID_PAGE)
	{ // stream is corrupted here ? should free everything ? TODO
		writeValToPage(s, (uint8_t *)&len, sizeof(len));
		writeValToPage(s, (uint8_t *)&txOrder, sizeof(txOrder));
		writeValToPage(s, (uint8_t *)&msgFlgs, sizeof(msgFlgs));
	} else {
		s->txMsgHdr.msgLen = len;
		s->txMsgHdr.msgFlgs = msgFlgs;
		s->txOrder = txOrder;
	}
}

void l4CmtRx(L4Pkt *l4Pkt, const uint8_t prio) { // recieved a frame
	L4Hdr *l4Hdr = &l4Pkt->hdr;
	// identify the stream
	stream_t *const s = &streams[l4Pkt->pos][prio];

	if (l4Hdr->msgFlgs & L4_MSG_FLAG_TYPE_ACK) // this is an ack message
	{
		if ((s->txMsgHdr.msgFlgs & L4_MSG_FLAG_PENDING_ACK) &&
			 s->txMsgHdr.msgNo == l4Hdr->msgNo)
		{ // message was successfully sent
			clearMsgFrame(NULL, s, true);
		}
	}
	else if (l4Hdr->msgLen == 0)
	{
		// call into app immediatly to process message
		appRecv(s);
		freeRxStream(s);

		if (l4Hdr->msgFlgs & L4_MSG_FLAG_REQ_ACK)
		{
			// tx an ack
			l4SndAck(s);
		}
	}
}

uint8_t getL4RxPktFrag(L4Pkt *l4Pkt, uint8_t **ptr, uint8_t rxLen, uint8_t prio)
{
	L4Hdr* l4Hdr = &l4Pkt->hdr;
	if (l4Hdr->msgFlgs & L4_MSG_FLAG_TYPE_ACK) // ack msg should have only hdr
	{
		return 0;
	}

	stream_t *const s = &streams[l4Pkt->pos][prio];
	uint8_t len;
	*ptr = getPgPtr(&s->rxPgPtr, &len, rxLen);
	return len;
}

bool l4CmtRxPnding(L4Pkt* l4Pkt) {
	L4Hdr* l4Hdr = &l4Pkt->hdr;
	return !(l4Hdr->msgFlgs & L4_MSG_FLAG_RXHD_CMT);
}

bool l4CmtRxHd(L4Pkt *l4Pkt, const uint8_t pos, const uint8_t prio)
{
	L4Hdr *l4Hdr = &l4Pkt->hdr;

	l4Pkt->pos = pos;
	stream_t *const s = &streams[l4Pkt->pos][prio];

	if (s->rxMsgHdr.msgNo != l4Hdr->msgNo) {
		freeRxStream(s); // clear the rx stream for new message
	}

	l4Hdr->msgFlgs |= L4_MSG_FLAG_RXHD_CMT;
	return true;
}
