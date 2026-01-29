//#include "layer4.h"
#include "layer3.h"
#include "allocator.h"
#include "layer2.h"
#include "network.h"

#define L2_FRAME_SIZE (RS485_FRAME_SIZE - (sizeof(L2Hdr) + sizeof(((L2Pkt*)0)->crc)))
#define L3_FRAME_SIZE (L2_FRAME_SIZE - sizeof(L3Hdr))
#define L4_FRAME_SIZE (L3_FRAME_SIZE - sizeof(L4Hdr))

#define L4_MAX_RETRY 3

stream_t streams[MAX_POS][MAX_PRIORITY];

TxOrderType txOrder = 0;

void l4Init() {
	for (int prio = 0; prio < MAX_PRIORITY; prio++) {
		for (int pos = 0; pos < MAX_POS; pos++) {
#if 0
			stream_t* s = &streams[pos];
			prio_stream_t* ps = &s->prio[prio];
#endif
			stream_t* s = &streams[pos][prio];
			s->txOrder = 0;
			memset(&s->txMsgHdr, 0x0, sizeof(L4Hdr));
			s->head_page = INVALID_PAGE;
			s->tail_page = INVALID_PAGE;
			s->head_off = 0;
			s->tail_used = 0;
			s->retryCnt = 0;
			s->retryTmr = 0;
		}
	}
}

uint8_t getL4PktHd(L4Pkt* l4Pkt, uint8_t* offset) {
	stream_t* s = &streams[l4Pkt->sIdx][l4Pkt->psIdx];
	*offset = s->head_off;
	return s->head_page;
}

void setL4Hdr(L4Pkt* l4Pkt) {
	L4Hdr* l4PktHdr = &l4Pkt->hdr;
	L4Hdr* txHdr = &streams[l4Pkt->sIdx][l4Pkt->psIdx].txMsgHdr;
	memcpy(l4PktHdr, txHdr, (sizeof(L4Hdr) - sizeof(MsgLenType)));
	l4PktHdr->msgLen = (txHdr->msgLen > L4_FRAME_SIZE) ? (txHdr->msgLen - L4_FRAME_SIZE) : 0; // remaining msg len
}

void getL4PktFrag(L4Pkt* l4Pkt, uint8_t** ptr, uint8_t* len, uint8_t* txHd, uint8_t* txHdOfst, uint8_t txLen) {
	const uint16_t base = pageOff(*txHd) + (uint16_t)(*txHdOfst);
	// start with first page
	//*ptr = &g_pool[base] + (*txHdOfst);
	//*len = (UNIT - (*txHdOfst));
	*ptr = &g_pool[base];

	const stream_t* s = &streams[l4Pkt->sIdx][l4Pkt->psIdx];

	// gate for msglen
	uint16_t ofst = pageOff(s->head_page) + (uint16_t)(s->head_off);
	uint16_t prioTxCnt = base - ofst;
	
	// cap tx len to min of msg len/Frame size/Fifo size
	*len = min((UNIT - (*txHdOfst)), 
			min((s->txMsgHdr.msgLen - prioTxCnt),
				min(txLen, (L4_FRAME_SIZE - prioTxCnt))));
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
		//return false;
	}
	else {
		*txHdOfst += *len;
	}

	uint16_t newTxLen = *len + prioTxCnt;

	if ((newTxLen >= s->txMsgHdr.msgLen) ||
		newTxLen >= L4_FRAME_SIZE) { // end tx
		*txHd = INVALID_PAGE;
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

void clearMsgFrame(L4Pkt* l4Pkt) {
	stream_t* s = &streams[l4Pkt->sIdx][l4Pkt->psIdx];
	L4Hdr* txHdr = &s->txMsgHdr;
	// free till Frame length
	uint8_t len = (txHdr->msgLen > L4_FRAME_SIZE) ? L4_FRAME_SIZE: txHdr->msgLen;
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
	L4Hdr* hdr = &l4Pkt->hdr;
	
	if (s->head_page != INVALID_PAGE) {

		if (hdr->msgLen == 0) {
			txHdr->msgNo++; // increment seq number

			for (int i = 0; i < sizeof(txHdr->msgLen); i++) {
				uint16_t base = pageOff(s->head_page) + (s->head_off);
				txHdr->msgLen |= (g_pool[base]) << (8 * i);
				s->head_off++;
				if (s->head_off == UNIT) {
					s->head_page = g_next[s->head_page];
					s->head_off = 0;
				}
			}

			if (txHdr->msgLen == 0) { //  no more message pending
				s->head_page = INVALID_PAGE;
				s->tail_page = INVALID_PAGE;
			}
			else { //  get tx order
				for (int i = 0; i < sizeof(s->txOrder); i++) {
					uint16_t base = pageOff(s->head_page) + (s->head_off);
					s->txOrder |= (g_pool[base]) << (8 * i);
					s->head_off++;
					if (s->head_off == UNIT) {
						s->head_page = g_next[s->head_page];
						s->head_off = 0;
					}
				}
			}
		}
		else {
			txHdr->msgLen = hdr->msgLen;
		}
	}
}

void l4TxCmplt(L4Pkt* l4Pkt) {
	 if (!(l4Pkt->hdr.msgFlgs & L4_MSG_FLAG_REQ_ACK)) { // this msg does not require an ack, frame can be cleared from page buffer
		 clearMsgFrame(l4Pkt);
	 }
}

// directly enqueue next prio msg in ACK
bool l4Ack(L4Pkt* l4Pkt, uint8_t prio, uint16_t dstAddr) {

#if 0
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
#endif
}

bool getL4Pkt(L4Pkt* l4Pkt, uint8_t pos, uint8_t prio) {
	TxOrderType currTxOrder = (l4Pkt->sIdx < MAX_POS)? streams[l4Pkt->sIdx][l4Pkt->psIdx].txOrder: ~0U;

	stream_t* ps = &streams[pos][prio];
	if (ps->head_page != INVALID_PAGE && 
		ps->txOrder <= currTxOrder) { // nothing to send
		l4Pkt->sIdx = pos;
		l4Pkt->psIdx = prio;
		return true;
	}

	return false;
}