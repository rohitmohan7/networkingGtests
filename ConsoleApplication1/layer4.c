//#include "layer4.h"
#include "layer3.h"
#include "allocator.h"
#include "layer2.h"
#include "network.h"
#include "pit.h"
#include "app.h"

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
			s->txOrder = ~0U;
			memset(&s->txMsgHdr, 0x0, sizeof(L4Hdr));
			memset(&s->rxMsgHdr, 0x0, sizeof(L4Hdr));
			s->rxMsgHdr.rxDesc.hd = INVALID_PAGE;
			pgPtrInit(&s->txPgPtr);
			
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
		*hd = INVALID_PAGE;
	//	*offset = 0;
		return false;
	}

	uint8_t currHd = s->txPgPtr.hdPg;
	uint8_t currOfst = s->txPgPtr.hdOfst;

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
	return false;
}

static inline void setL4HdrMsgNoMsgLen(L4Hdr* l4PktHdr, L4Hdr* txHdr, uint8_t txFrameCnt) {
	l4PktHdr->msgNo = txHdr->msgNo;
	l4PktHdr->msgLen = ((txHdr->msgLen - (txFrameCnt * L4_FRAME_SIZE)) > L4_FRAME_SIZE) ?
		((txHdr->msgLen - (txFrameCnt * L4_FRAME_SIZE)) - L4_FRAME_SIZE) : 0;
}

void setL4HdrBrdcst(L4Hdr* l4PktHdr, L4Hdr* txHdr) {
	setL4HdrMsgNoMsgLen(l4PktHdr, txHdr, 0); // no tx frame count
}

void setL4Hdr(L4Pkt* l4Pkt, uint8_t prio) {
	L4Hdr* l4PktHdr = &l4Pkt->hdr;
	stream_t* ps = &streams[l4Pkt->pos][prio];
	L4Hdr* txHdr = &ps->txMsgHdr;
	setL4HdrMsgNoMsgLen(l4PktHdr, txHdr, ps->txFrameCnt);
	l4PktHdr->msgFlgs = (txHdr->msgFlgs & ~(L4_MSG_FLAG_STREAM_PENDING | L4_MSG_FLAG_PENDING_ACK)); // unset internal flag
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

static inline void freeStream(stream_t *s)
{
	freePgPtr(&s->txPgPtr);
	s->txMsgHdr.msgFlgs = 0;
}

static inline void freeRxStream(stream_t *s)
{
	freePgPtr(&s->rxPgPtr);
	//s->rxMsgHdr.msgFlgs = 0;
	//s->rxMsgHdr.rxDesc.hd = INVALID_PAGE;
}

void l4SetBrdcastStrmPnding(L4Hdr * hdr, const bool pending) {
	if (pending) {
		hdr->msgFlgs |= L4_MSG_FLAG_STREAM_PENDING;
	} else {
		hdr->msgFlgs &= ~L4_MSG_FLAG_STREAM_PENDING;
	}
}

static inline bool clearMsgFrame(L4Pkt *l4Pkt, stream_t *s, uint8_t pos, bool allFrames)
{
	PgPtr_t* pgPtr = &s->txPgPtr;
	L4Hdr* txHdr = &s->txMsgHdr;
	L4Hdr *hdr = l4Pkt? &l4Pkt->hdr: NULL;
	// free till Frame length
	uint8_t len = (!allFrames && txHdr->msgLen > L4_FRAME_SIZE) ? L4_FRAME_SIZE: 
		(txHdr->msgLen + ((allFrames && hdr)? hdr->msgLen:0));
	freePgPtrLen(pgPtr, len);

	// advance stream
	if (pgPtr->hdPg != INVALID_PAGE) {

		if (allFrames || !hdr || hdr->msgLen == 0) {
			txHdr->msgNo++; // increment seq number
#if 0 /* TODO Check why below dosnt work */
			static const uint8_t msgNumLen = sizeof(uint8_t);
			readFromPgs(pgPtr, ((uint8_t*)txHdr) + msgNumLen, sizeof(txHdr->msgFlgs) + sizeof(txHdr->msgLen));
#endif
			
			readFromPgs(pgPtr, (uint8_t*)&txHdr->msgFlgs, sizeof(txHdr->msgFlgs));
			readFromPgs(pgPtr, (uint8_t*)&txHdr->msgLen, sizeof(txHdr->msgLen));
			readFromPgs(pgPtr, (uint8_t*)&s->txOrder, sizeof(TxOrderType));

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
			//txHdr->msgLen = hdr->msgLen;
			txHdr->msgLen -= len;
			s->txMsgHdr.msgFlgs |= L4_MSG_FLAG_STREAM_PENDING;
		}
		return false;
	}
	return true;
}

bool l4lstBrdcstMsgFrm(L4Hdr* l4Pkt) {
	bool ret = l4Pkt->msgLen < L4_FRAME_SIZE;
}

bool l4TxBrdcstStrmPnding(L4Hdr* l4Hdr) {
	return (l4Hdr->msgFlgs & L4_MSG_FLAG_STREAM_PENDING);
}

bool l4lstMsgFrm(uint16_t pos, uint8_t prio) {
	stream_t* ps = &streams[pos][prio];
	return (ps->txMsgHdr.msgLen < L4_FRAME_SIZE) || (ps->txMsgHdr.msgLen - (ps->txFrameCnt * L4_FRAME_SIZE)) < L4_FRAME_SIZE;
}

void l4TxCmplt(L4Pkt* l4Pkt, uint8_t prio) {
	stream_t *s = &streams[l4Pkt->pos][prio];
	if (!(l4Pkt->hdr.msgFlgs & L4_MSG_FLAG_REQ_ACK)) { 
		// this msg does not require an ack, frame can be cleared from page buffer
		clearMsgFrame(l4Pkt, s, l4Pkt->pos, false);
	} else {
		 
		 if (l4lstMsgFrm(l4Pkt->pos, prio)) {
			 s->txMsgHdr.msgFlgs |= L4_MSG_FLAG_PENDING_ACK;
			 //set the timer
			 s->retryTmr = pitGetCurrMS();
		 }
		 s->txFrameCnt++;
	 }
}

static inline bool l4StrmEmptyAftFrme(const PgPtr_t* const pgPtr, const MsgLenType_t msgLen) {
	/* make a copy of pgPtr */
	PgPtr_t pgPtrCpy = *pgPtr;

	/* advance by msg len or frame size */
	advancePgPtrLen(&pgPtrCpy, min(msgLen, L4_FRAME_SIZE));

	return pgPtrCpy.hdPg == INVALID_PAGE; // these is still message after this frame or msglen
}

bool l4StrmEmptyAftBrdcstFrme(const PgPtr_t* const pgPtr, const MsgLenType_t msgLen) {
	return l4StrmEmptyAftFrme(pgPtr, msgLen);
}

bool l4StrmEmptyAftUnicstFrme(uint16_t pos, uint8_t prio) {
	stream_t* s = &streams[pos][prio];

	if ((s->txMsgHdr.msgFlgs & L4_MSG_FLAG_REQ_ACK) && l4lstMsgFrm(pos, prio)) {
		// after last frame this stream will wait for ack before moving to next message, consider empty after frame...
		return true;
	}

	return l4StrmEmptyAftFrme(&s->txPgPtr, s->txMsgHdr.msgNo);
}

TxOrderType l4GetStrmTxOrder(const uint8_t pos, const uint8_t prio) {
	stream_t* s = &streams[pos][prio];
	return s->txOrder;
}

bool l4TxBrdcastStrmEmpty(PgPtr_t * pgPtr) {
	return pgPtr->hdPg == INVALID_PAGE;
}

bool l4StrmEmpty(uint16_t pos, uint8_t prio) {
	stream_t* ps = &streams[pos][prio];
	if (ps->txPgPtr.hdPg == INVALID_PAGE) {
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

	/* TODO check why ? <= than < ? */

	if ((ps->txPgPtr.hdPg != INVALID_PAGE || ps->txMsgHdr.msgFlgs & L4_MSG_FLAG_TYPE_ACK) &&
		ps->txOrder <= currTxOrder)
	{ // nothing to send

		if ((ps->txMsgHdr.msgFlgs & L4_MSG_FLAG_PENDING_ACK))
		{ // check if this message has timed out
			if (pitTimespanExceeded(ps->retryTmr, pitGetCurrMS(), L4_RETRY_TIMER))
			{
				if (ps->retryCnt >= MAX_L4_RETRY)
				{
					// drop move and move to next msg
					const bool strmEmpty = clearMsgFrame(l4Pkt, ps, pos, true);
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

void writeValToPage(PgPtr_t * pgPtr, uint8_t *val, uint8_t len)
{
	while (len) {
		uint8_t writeLen;
		uint8_t* ptr = getPgPtr(pgPtr, &writeLen, len);
		writeLen = min(len, writeLen);
		memcpy(ptr, val, writeLen);
		val += writeLen;
		len -= writeLen;
	}
}

void l4SndAck(stream_t *const s)
{
	MsgLenType_t len = 0;
	uint8_t msgFlgs = L4_MSG_FLAG_TYPE_ACK;
	PgPtr_t* pgPtr = &s->txPgPtr;
	if (pgPtr->tlPg != INVALID_PAGE)
	{ // stream is corrupted here ? should free everything ? TODO
		writeValToPage(pgPtr, (uint8_t *)&len, sizeof(len));
		writeValToPage(pgPtr, (uint8_t *)&txOrder, sizeof(txOrder));
		writeValToPage(pgPtr, (uint8_t *)&msgFlgs, sizeof(msgFlgs));
	} else {
		s->txMsgHdr.msgLen = len;
		s->txMsgHdr.msgFlgs = msgFlgs;
		s->txOrder = txOrder;
	}
}

#define MAX_UDP_DATAGRAM_SIZE 1000

void l4CmtRx(PgPtr_t* const pgPtr, const Protocol_t proto, MsgLenType_t msgLen) { // recieved a frame

	switch (proto) {
	case IP_PROTO_UDP:
		if (msgLen > sizeof(UdpHdr_t)) {
			msgLen -= sizeof(UdpHdr_t);
			if (msgLen <= MAX_UDP_DATAGRAM_SIZE) {
				/* first read udp header */
				UdpHdr_t udpHdr;
				readFromPgs(pgPtr, (uint8_t*)&udpHdr, sizeof(UdpHdr_t));

				if (udpHdr.msgLen == msgLen) {
					uint8_t udpData[MAX_UDP_DATAGRAM_SIZE];
					readFromPgs(pgPtr, udpData, msgLen);
					appRecv(udpHdr.dstPort, udpData, msgLen);
				}
			}
		}
		freePgPtr(pgPtr);
		break;
	default:
		break;
	}

#if 0
	L4Hdr *l4Hdr = &l4Pkt->hdr;
	// identify the stream
	stream_t *const s = &streams[l4Pkt->pos][prio];

	if (l4Hdr->msgFlgs & L4_MSG_FLAG_TYPE_ACK) // this is an ack message
	{
		if ((s->txMsgHdr.msgFlgs & L4_MSG_FLAG_PENDING_ACK) &&
			 s->txMsgHdr.msgNo == l4Hdr->msgNo)
		{ // message was successfully sent
			clearMsgFrame(NULL, s, l4Pkt->pos, true);
		}
	}
	else if (l4Hdr->msgLen == 0)
	{
		// call into app immediatly to process message
		appRecv(s);
		freeRxStream(s);

		if (l4Hdr->msgFlgs & L4_MSG_FLAG_REQ_ACK) {
			// tx an ack
			l4SndAck(s);
		}
	}
	
	s->rxMsgHdr.rxDesc.hd = INVALID_PAGE;
#endif
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
	/* Only checked once post rx */
	L4Hdr* l4Hdr = &l4Pkt->hdr;
	uint8_t msgFlgs = l4Hdr->msgFlgs;
	//l4Hdr->msgFlgs &= ~L4_MSG_FLAG_RXHD_CMT; /* free incase needed to brdcast */
	return !(msgFlgs & L4_MSG_FLAG_RXHD_CMT);
}

void l4CmtRxHd(L4Pkt *l4Pkt, const uint8_t pos, const uint8_t prio)
{
	L4Hdr *l4Hdr = &l4Pkt->hdr;

	l4Pkt->pos = pos;
	stream_t *const s = &streams[l4Pkt->pos][prio];

	if (s->rxMsgHdr.msgNo != l4Hdr->msgNo) {
		freeRxStream(s); // clear the rx stream for new message
		s->rxMsgHdr.msgNo = l4Hdr->msgNo;
	}

	// keep track of prev head
	s->rxMsgHdr.rxDesc.hd = s->rxPgPtr.tlPg;
	s->rxMsgHdr.rxDesc.hdOfst = s->rxPgPtr.tlUsd;

	l4Hdr->msgFlgs |= L4_MSG_FLAG_RXHD_CMT;
}

void l4AbortRx(L4Pkt* l4Pkt, const uint8_t prio) {
	// clear current frame
	stream_t* const s = &streams[l4Pkt->pos][prio];
	const uint8_t frmHd = s->rxMsgHdr.rxDesc.hd;

	if (frmHd != INVALID_PAGE) {
		// clear frame
		if (frmHd == s->rxPgPtr.hdPg) {
			// rx stream will always start with hd offset 0
			freeRxStream(s);
		} else {
			PgPtr_t frmPtr;
			frmPtr.hdPg = g_next[frmHd];
			freePgPtr(&frmPtr);

			// set new stream tail
			s->rxPgPtr.tlPg = frmHd;
			s->rxPgPtr.tlUsd = s->rxMsgHdr.rxDesc.hdOfst;
		}
	}
}

void l4RxGetLastFrame(const uint8_t prio, L4Pkt *l4Pkt, PgPtr_t *frame)
{
	stream_t *const s = &streams[l4Pkt->pos][prio];
	
	if (s->rxMsgHdr.rxDesc.hd == INVALID_PAGE) {
		/* First frame */
		frame->hdPg = s->rxPgPtr.hdPg;
		frame->hdOfst = s->rxPgPtr.hdOfst;
	} else {
		frame->hdPg = s->rxMsgHdr.rxDesc.hd;
		frame->hdOfst = s->rxMsgHdr.rxDesc.hdOfst;
	}
		
	frame->tlPg = s->rxPgPtr.tlPg;
	frame->tlUsd = s->rxPgPtr.tlUsd;
	addUser(frame);
}

TxOrderType l4GetTxOrder(const uint8_t pos, const uint8_t prio) {
	const stream_t* const s = &streams[pos][prio];
	return s->txOrder;
}
