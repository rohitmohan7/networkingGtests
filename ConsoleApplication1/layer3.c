#include "layer2.h"
#include "layer3.h"
//#include "layer4.h"
#include "network.h"

#define L3_DST_ADDR_IDX 0
#define L3_ADDR_SUBNET_SHIFT 8

#define L3_FRAME_SIZE (L2_FRAME_SIZE - sizeof(L3Hdr))
#define L4_FRAME_SIZE (L3_FRAME_SIZE - sizeof(L4Hdr))

uint16_t l3AddrTblPrio[MAX_POS][MAX_PORT]; // pos addresses ordered by tx priority
uint16_t l3RouteTable[MAX_SUBNET]; // next best gateway for subnet
uint8_t l3BcastInSubnetForSrcPort[MAX_POS][MAX_PORT];
uint8_t l3RouteHops[MAX_SUBNET]; // l3 hop table for my pos

#define LOW_PRIO_IDX 1
#define BROADCAST_ADDR 0x0000

static inline bool l3TxBrdcstPkt(const L3Hdr *const l3Hdr)
{
	return l3Hdr->dst == BROADCAST_ADDR;
}

static inline bool l3Multicst(const L3Hdr *const l3Hdr, const uint8_t port)
{
	const uint8_t portSubnet = l3AddrTblPrio[myPos][port] >> 8;
	const uint8_t dstSubnet = l3Hdr->dst >> 8;
	return (((uint8_t)l3Hdr->dst) == 0x00) && (portSubnet == dstSubnet);
}

l3BrdCstStrm_t l3BrdcstStrms[MAX_PORT][MAX_PRIORITY];

static inline uint8_t getL3PktFragInternal(PgPtr_t* frwdPgPtr, uint8_t** ptr, uint8_t len, uint8_t* txHd, uint8_t* txHdOfst, uint8_t txLen) {
	const uint16_t base = pageOff(*txHd) + (uint16_t)(*txHdOfst);

	*ptr = &g_pool[base];

	if (len <= txLen)
	{ // we are at the end of current page
		*txHd = g_next[(*txHd)];
		*txHdOfst = 0;
	}
	else
	{
		len = min(txLen, len);
		*txHdOfst += len;
	}

	if ((*txHd == frwdPgPtr->tlPg &&
		frwdPgPtr->tlUsd == *txHdOfst)) {
		*txHd = INVALID_PAGE;
	}

	return len;
}

static inline uint8_t getL3BrdCstPktFrag(l3BrdCstStrm_t* brdcstStrm, uint8_t** ptr, uint8_t idx, uint8_t* txHd, uint8_t* txHdOfst, uint8_t txLen) {
	uint8_t len = min((UNIT - (*txHdOfst)), (brdcstStrm->txMsgHdr.msgLen - idx));
	uint8_t ret = getL3PktFragInternal(&brdcstStrm->txPgPtr, ptr, len, txHd, txHdOfst, txLen);
	if (len + idx >= brdcstStrm->txMsgHdr.msgLen) {
		*txHd = INVALID_PAGE;
	}
	return ret;
}

#if MAX_PORT > 1 /* Forwarding */
#define MAX_FORWARD_QUEUE 4 // power of 2
#define MAX_FORWARD_QUEUE_MASK (MAX_FORWARD_QUEUE - 1u)
L3Pkt l3FrwdQ[MAX_PORT][MAX_PRIORITY][MAX_FORWARD_QUEUE];
PgPtr_t l3FrwdPgPtrQ[MAX_PORT][MAX_PRIORITY][MAX_FORWARD_QUEUE];
/* Keep one page ptr for the RX Frwd Pkt */
PgPtr_t l3RxFrwdPktPgPtr[MAX_PORT];

static uint8_t l3FrwdQHead[MAX_PORT][MAX_PRIORITY];
static uint8_t l3FrwdQTail[MAX_PORT][MAX_PRIORITY];
static uint8_t l3FrwdQCount[MAX_PORT][MAX_PRIORITY];

static inline void l3FrwdQInit(void)
{
	memset(l3FrwdQ, 0, sizeof(l3FrwdQ));

	// invalidate the page ptrs
	for (uint8_t port = 0; port < MAX_PORT; port++)
	{
		pgPtrInit(&l3RxFrwdPktPgPtr[port]);
		for (uint8_t prio = 0; prio < MAX_PRIORITY; prio++)
		{
			pgPtrInit(&l3BrdcstStrms[port][prio].txPgPtr);
			for (uint8_t queueIdx = 0; queueIdx < MAX_FORWARD_QUEUE; queueIdx++)
			{
				pgPtrInit(&l3FrwdPgPtrQ[port][prio][queueIdx]);
				memset(l3FrwdQ[MAX_PORT][MAX_PRIORITY][MAX_FORWARD_QUEUE].frwdPkt.data, 0, sizeof(L4Pkt));
			}
		}
	}

	memset(l3FrwdQHead, 0, sizeof(l3FrwdQHead));
	memset(l3FrwdQTail, 0, sizeof(l3FrwdQTail));
	memset(l3FrwdQCount, 0, sizeof(l3FrwdQCount));
}

static inline bool getL3FrwdPktHd(const uint8_t port, const uint8_t prio, uint8_t *hd, uint8_t *ofst)
{
	const uint8_t frwdQHd = l3FrwdQHead[port][prio];
	PgPtr_t *frwdPgPtr = &l3FrwdPgPtrQ[port][prio][frwdQHd];
	if (frwdPgPtr->hdPg != INVALID_PAGE)
	{ // could be message with just a hdr
		*ofst = frwdPgPtr->hdOfst;
		*hd = frwdPgPtr->hdPg;
		return false;
	}
	return true;
}

static inline void l3FwdTxCmplt(L3Pkt *l3Pkt, const uint8_t port)
{
	const uint8_t prio = l3Pkt->hdr.prio;
	// free the allocated pages
	const uint8_t frwdQHd = l3FrwdQHead[port][prio];
	freePgPtr(&l3FrwdPgPtrQ[port][prio][frwdQHd]);
	// free the forward queue
	l3FrwdQHead[port][prio] = (uint8_t)((l3FrwdQHead[port][prio] + 1u) % MAX_FORWARD_QUEUE);
	l3FrwdQCount[port][prio]--;
}

static inline bool l3RxfrwdPkt(const L3Hdr *const l3Hdr, const uint8_t port)
{
	// dst addr is not to this device port
	const bool unicast = l3Hdr->dst == l3AddrTblPrio[myPos][port];
	return !l3TxBrdcstPkt(l3Hdr) && !l3Multicst(l3Hdr, port) && !unicast;
}

static inline bool l3TxfrwdPkt(const L3Hdr *const l3Hdr, const uint8_t port)
{
	// src addr is not from this device port
	return l3Hdr->src != l3AddrTblPrio[myPos][port];
}

bool passMst(uint8_t port, uint8_t prioIdx, uint16_t pos);

static inline bool passMstFrwd(const uint8_t port, const uint8_t prioIdx)
{
	if (prioIdx < LOW_PRIO_IDX &&
		l3FrwdQCount[port][prioIdx] > 1) // more high prio frwd pkts waiting
	{
		return false;
	}

	return passMst(port, prioIdx, 0); // check from pos 0 of this prio if there is a pending high prio stream or frwd pkt
}

static inline uint8_t getL3FrwdPktFrag(PgPtr_t *frwdPgPtr, uint8_t **ptr, uint8_t idx, uint8_t *txHd, uint8_t *txHdOfst, uint8_t txLen)
{
	const uint16_t base = pageOff(*txHd) + (uint16_t)(*txHdOfst);

	*ptr = &g_pool[base];

	// cap tx len
	uint8_t len = (*txHd == frwdPgPtr->tlPg ? (frwdPgPtr->tlUsd - *txHdOfst) : (UNIT - (*txHdOfst)));

	if (len <= txLen)
	{ // we are at the end of current page
		*txHd = g_next[(*txHd)];
		*txHdOfst = 0;
	}
	else
	{
		len = min(txLen, len);
		*txHdOfst += len;
	}

	if ((*txHd == frwdPgPtr->tlPg && 
		frwdPgPtr->tlUsd == *txHdOfst)) {
		*txHd = INVALID_PAGE;
	}

	return len;
}

static inline uint8_t getL3RxFrwdPktFrag(PgPtr_t *const frwdPktPtr, uint8_t **ptr, uint8_t rxLen)
{
	uint8_t len;
	*ptr = getPgPtr(frwdPktPtr, &len, rxLen);
	return len;
}

static inline PgPtr_t * l3PushFrwdPkt(L3Pkt *const l3Pkt, const uint8_t port)
{
	const uint8_t prio = l3Pkt->hdr.prio;

	if (l3FrwdQCount[port][prio] >= MAX_FORWARD_QUEUE) {
		// TODO do we tx L3 forward Queue full or prio error?
		return NULL; /* full */
	}
	
	const uint8_t frwdQIdx = l3FrwdQTail[port][prio];
	L3Pkt *l3FrwdPkt = &l3FrwdQ[port][prio][frwdQIdx];
	
	/* Copy the header */
	memcpy(l3FrwdPkt, l3Pkt, sizeof(L3Hdr) + sizeof(L4Hdr));

	l3FrwdQTail[port][prio] = (uint8_t)((l3FrwdQTail[port][prio] + 1u) % MAX_FORWARD_QUEUE);
	l3FrwdQCount[port][prio]++;
	return &l3FrwdPgPtrQ[port][prio][frwdQIdx];
}

static inline void l3BrdCst(const uint8_t port, L3Pkt *const l3Pkt)
{
	const L3Hdr *l3Hdr = &l3Pkt->hdr;
	const uint8_t portSubnet = l3AddrTblPrio[myPos][port] >> 8;
	const uint8_t prio = l3Hdr->prio;

	for (int peerPort = 0; peerPort < MAX_PORT; peerPort++)
	{
		if (peerPort == port)
		{
			continue;
		}

		// check brdcast table
		if (l3BcastInSubnetForSrcPort[l3Pkt->l4Pkt.pos][peerPort] == portSubnet)
		{
			PgPtr_t *frwdQPgPtr = l3PushFrwdPkt(l3Pkt, peerPort);
			if (frwdQPgPtr) {
				l4RxGetLastFrame(prio, &l3Pkt->l4Pkt, frwdQPgPtr);
			} else {
				/* Todo do we xmit to sender ? */
			}
		}
	}
}

#endif

uint8_t l3GetTxPktHdrSize(L3Pkt *l3Pkt, uint8_t port)
{
	L3Hdr *l3Hdr = &l3Pkt->hdr;
//#if 0
#if MAX_PORT > 1
	if (l3TxfrwdPkt(l3Hdr, port))
	{
		return sizeof(L3Hdr);
	}
#endif
//#endif
	return sizeof(L3Hdr) + sizeof(L4Hdr);
}

uint8_t l3GetRxPktHdrSize(L3Pkt *l3Pkt, uint8_t port)
{
	L3Hdr *l3Hdr = &l3Pkt->hdr;
//#if 0
#if MAX_PORT > 1
	if (l3RxfrwdPkt(l3Hdr, port))
	{
		return sizeof(L3Hdr); // forward packet
	}
#endif
//#endif
	return sizeof(L3Hdr) + sizeof(L4Hdr);
}

bool l3TxBrdcstMsg(const uint8_t* data, MsgLenType len, uint8_t priority) {
	/* Critical Section: Function should be gated against interrupt from RS485 UART or PIT to avoid data sync issues */
	PgPtr_t * pgPtr = NULL;
	PgPtrTl_t pgPtrTl;
	uint8_t port = 0;

	for (; port < MAX_PORT; port++) {
		l3BrdCstStrm_t* const brdcstStream = &l3BrdcstStrms[port][priority];
		if (brdcstStream->txPgPtr.tlPg != INVALID_PAGE) { /* found a still active brdcast stream append to the tail*/
			pgPtr = &brdcstStream->txPgPtr;
			
			/* keep copy of current tail */
			pgPtrTl.tl = pgPtr->tlPg;
			pgPtrTl.tlUsd = pgPtr->tlUsd;
			//memcpy(&pgPtrTl, pgPtr + sizeof(PgPtrHd_t), sizeof(PgPtrTl_t));

			/* write len and tx order */
			writeValToPage(pgPtr, (uint8_t*)&len, sizeof(len));
			writeValToPage(pgPtr, (uint8_t*)&txOrder, sizeof(txOrder));

			/* write data */
			writeValToPage(pgPtr, data, len);
			break;
		}
	}

	if (!pgPtr) {
		/* no active broadcast streams */
		for (uint8_t port = 0; port < MAX_PORT; port++) {
			if (l3AddrTblPrio[myPos][port] == 0) { /* what if route table is not initialized yet ? */
				continue;
			}

			l3BrdCstStrm_t* const brdcstStream = &l3BrdcstStrms[port][priority];
			brdcstStream->txOrder = txOrder;
			brdcstStream->txMsgHdr.msgLen = len;

			if (!pgPtr) {
				pgPtr = &brdcstStream->txPgPtr;
				/* write data */
				writeValToPage(pgPtr, data, len);
			}
			else {

				/* mem cpy the pg ptrs */
				memcpy(&brdcstStream->txPgPtr, pgPtr, sizeof(PgPtr_t));
				addUser(pgPtr);
			}
		}
	} else { // these is an active broadcast stream we need to use its tail
		for (uint8_t peerPort = 0; peerPort < MAX_PORT; peerPort++) {
			if (peerPort == port) {
				continue;
			}

			l3BrdCstStrm_t* const brdcstStream = &l3BrdcstStrms[peerPort][priority];
			

			if (brdcstStream->txPgPtr.tlPg == INVALID_PAGE) {
				/* this stream is finished */
				brdcstStream->txOrder = txOrder;
				brdcstStream->txMsgHdr.msgLen = len;
				/* */
				memcpy(&brdcstStream->txPgPtr, &pgPtrTl, sizeof(PgPtrHd_t));
				//memcpy(&brdcstStream->txPgPtr + sizeof(PgPtrHd_t), &pgPtr, sizeof(PgPtrTl_t));
				brdcstStream->txPgPtr.tlPg = pgPtr->tlPg;
				brdcstStream->txPgPtr.tlUsd = pgPtr->tlUsd;

				advancePgPtrLen(&brdcstStream->txPgPtr, sizeof(len) + sizeof(txOrder));
				addUser(&brdcstStream->txPgPtr);
			} else { /* extend its tail */
				addUser(pgPtr);
				memcpy(&brdcstStream->txPgPtr + sizeof(PgPtrHd_t), &pgPtr, sizeof(PgPtrTl_t));
			}
 		}
	}

	return pgPtr != NULL;
}

void l3Init(void) {
#if MAX_PORT > 1
	l3FrwdQInit();
#endif
}

static inline void l3TxCmpltBrdcstPkt(const L3Pkt* const l3Pkt, const uint8_t port) {
	const L3Hdr* const l3Hdr = &l3Pkt->hdr;
	const l3BrdCstStrm_t* const brdcstStream = &l3BrdcstStrms[port][l3Hdr->prio];
	L4Hdr* txHdr = &brdcstStream->txMsgHdr;

	// free till Frame length
	uint8_t len = (txHdr->msgLen > L4_FRAME_SIZE) ? L4_FRAME_SIZE : txHdr->msgLen;
	const PgPtr_t* const pgPtr = &brdcstStream->txPgPtr;
	freePgPtrLen(pgPtr, len);
	
	bool brdcstStreamPending = false;

	if (pgPtr->hdPg != INVALID_PAGE) {
		if (l4lstBrdcstMsgFrm(txHdr)) {
			readFromPgs(pgPtr, (uint8_t*)&txHdr->msgLen, sizeof(txHdr->msgLen));
			readFromPgs(pgPtr, (uint8_t*)&brdcstStream->txOrder, sizeof(TxOrderType));

			if (!txHdr->msgLen) {
				freePgPtr(pgPtr);
			}
		} else {
			//txHdr->msgLen = l3Pkt->l4Pkt.hdr.msgLen;
			txHdr->msgLen -= len;
			brdcstStreamPending = true;
		}
	}

	l4SetBrdcastStrmPnding(txHdr, brdcstStreamPending);
}

void l3TxCmplt(L3Pkt* l3Pkt, const uint8_t port) {
	L3Hdr* l3Hdr = &l3Pkt->hdr;

#if MAX_PORT > 1
	if (l3TxfrwdPkt(l3Hdr, port)) {
		l3FwdTxCmplt(l3Pkt, port);
		return;
	}
#endif
	if (l3TxBrdcstPkt(l3Hdr)) {
		l3TxCmpltBrdcstPkt(l3Pkt, port);
	} else {
		l4TxCmplt(&l3Pkt->l4Pkt, l3Hdr->prio);
	}
}

static inline bool getL3BrdcstPktHd(const uint8_t port, const uint8_t prio, uint8_t* hd, uint8_t* ofst) {
	const l3BrdCstStrm_t* const brdcstStream = &l3BrdcstStrms[port][prio];
	*ofst = brdcstStream->txPgPtr.hdOfst;
	*hd = brdcstStream->txPgPtr.hdPg;
	return false;
}

bool getL3PktHd(L3Pkt *l3Pkt, uint8_t *hd, uint8_t *ofst, uint8_t port) {
	L3Hdr* l3Hdr = &l3Pkt->hdr;

#if MAX_PORT > 1
	if (l3TxfrwdPkt(l3Hdr, port)) {
		return getL3FrwdPktHd(port, l3Hdr->prio, hd, ofst);
	}
#endif

	/* check if its own broadcast */
	if (l3TxBrdcstPkt(l3Hdr)) {
		return getL3BrdcstPktHd(port, l3Hdr->prio, hd, ofst);
	}

	return getL4PktHd(&l3Pkt->l4Pkt, l3Hdr->prio, hd, ofst);
}

void l3AbortRx(L3Pkt* const l3Pkt, const uint8_t port) {
	L3Hdr* l3Hdr = &l3Pkt->hdr;

#if MAX_PORT > 1
	if (l3TxfrwdPkt(l3Hdr, port)) {
		freePgPtr(&l3RxFrwdPktPgPtr[port]);
		return;
	}
#endif

	l4AbortRx(&l3Pkt->l4Pkt, l3Hdr->prio);
}

static inline void setL3Hdr(L3Hdr* const l3Hdr, const uint16_t src, const uint8_t prio, uint16_t dstAddr) {
	l3Hdr->src = src;
	l3Hdr->ttl = 1;
	l3Hdr->prio = prio;
	l3Hdr->dst = dstAddr;
}

uint8_t setL3HdrBrdcst(L3Pkt * const l3Pkt, uint8_t port, uint8_t prio, uint16_t pos) {
	L3Hdr * const l3Hdr = &l3Pkt->hdr;
	setL3Hdr(l3Hdr, l3AddrTblPrio[myPos][port], prio, 0x0000);
	l3Hdr->ttl = 1;
	setL4HdrBrdcst(&l3Pkt->l4Pkt, &l3Pkt->l4Pkt.hdr);
	return 0;
}

uint8_t setL3HdrUnicst(L3Pkt * const l3Pkt, uint8_t port, uint8_t prio, uint16_t pos) {
	L3Hdr * const l3Hdr = &l3Pkt->hdr;
	setL3Hdr(l3Hdr, l3AddrTblPrio[myPos][port], prio, l3AddrTblPrio[pos][L3_DST_ADDR_IDX]);
	l3Hdr->ttl = 1;
	setL4Hdr(&l3Pkt->l4Pkt, prio);
	const uint8_t dstSubnet = (l3Hdr->dst & 0xFF00) >> 8;
	const uint8_t portSubnet = ((l3Hdr->src & 0xFF00) >> 8);
	return ((dstSubnet == portSubnet)? l3Hdr->dst: l3RouteTable[dstSubnet]) & 0x00FF; // return l2Addr
}

bool passMst(uint8_t port, uint8_t prioIdx, uint16_t pos) {

	for (uint8_t prio = prioIdx; prio < MAX_PRIORITY; prio++)
	{
		if (prio != prioIdx) {
#if MAX_PORT > 1
			/* Dont check frwd stream for current prioIdx (already checked!)*/
			if ((prio < LOW_PRIO_IDX) && l3FrwdQCount[port][prio])
			{
				return false;
			}
#endif
			/* check for pending broadcast stream */
			const l3BrdCstStrm_t* const brdcstStream = &l3BrdcstStrms[port][prio];
			if (prio >= LOW_PRIO_IDX) {
				if (l4TxBrdcstStrmPnding(&brdcstStream->txMsgHdr)) {
					return false;
				}
			} else if (!l4TxBrdcastStrmEmpty(&brdcstStream->txPgPtr)) {
				return false;
			}
		}

		for (; pos < MAX_POS; pos++)
		{
			if (prio >= LOW_PRIO_IDX)
			{ // check for pending low prio
				if (l4StrmPnding(pos, prio))
				{
					return false;
				}
			}
			else if (!l4StrmEmpty(pos, prio))
			{
				return false;
			}
		}
		pos = 0;
	}
	return true;
}


bool passMstStream(const uint8_t port, uint8_t prioIdx, uint16_t posIdx, const l3BrdCstStrm_t* const brdcstStream) {

	if (prioIdx < LOW_PRIO_IDX)
	{
		if (!brdcstStream) {
			if (!l4StrmEmptyAftUnicstFrme(posIdx, prioIdx)) {
				return false;
			}
		} else {
			if (!l4StrmEmptyAftBrdcstFrme(&brdcstStream->txPgPtr, brdcstStream->txMsgHdr.msgLen)) {
				return false;
			}
		}
		
		/* check other streams for prio msg and low prio pending */
		if (posIdx + 1 >= MAX_POS)
		{
			posIdx = 0;
			prioIdx++; // check from next priority
		}
		else
		{
			posIdx++; // check from next pos
		}

		return passMst(port, prioIdx, posIdx);
	} else{
		// if this the last frame of a low priority message pass the MST token
		if (brdcstStream) {
			return l4lstBrdcstMsgFrm(&brdcstStream->txMsgHdr);
		} else { // this is brdcast stream
			return l4lstMsgFrm(posIdx, prioIdx); 
		}
	}	
}

l3BrdCstStrm_t* const l3premptLowPrioPending(const uint8_t port, L3Pkt* l3Pkt, uint16_t* posIdx, uint8_t* prioIdx) {
	*prioIdx++; // txOrder premting would have already selected a pending stream
	L3Hdr* l3Hdr = &l3Pkt->hdr;

	for (uint8_t prio = *prioIdx; prio < MAX_PRIORITY; prio++) {
		/* check broad cast stream */
		const l3BrdCstStrm_t* const brdcstStream = &l3BrdcstStrms[port][prio];

		if (l4TxBrdcstStrmPnding(&brdcstStream->txMsgHdr)) {
			return brdcstStream;
		}

		for (int pos = 0; pos < MAX_POS; pos++) {
			if (l4StrmPnding(pos, prio)) { // prempt with this stream
				if (getL4Pkt(&l3Pkt->l4Pkt, pos, prio, l3Hdr->prio)) {
					*posIdx = pos;
					*prioIdx = prio;
					return NULL;
				} // this is an error condition should not get here
			}
		}
	}
	return NULL;
}

bool getl3Pkt(uint8_t port, L3Pkt* l3Pkt, bool* xferMst, uint8_t * l2Addr) {
	*xferMst = false;
	const uint8_t portSubnet = ((l3AddrTblPrio[myPos][port] & 0xFF00) >> 8);
	L3Hdr* l3Hdr = &l3Pkt->hdr;

	for (uint8_t prio = 0; prio < MAX_PRIORITY; prio++) {
		uint8_t pos = 0;
		
#if MAX_PORT > 1
		// check forward packet first 
		if (l3FrwdQCount[port][prio])
		{
			/* TODO what if we are still being filled from other port */
			
			/* Copy the header */
			const uint8_t frwdQHd = l3FrwdQHead[port][prio];
			L3Pkt *l3FrwdPkt = &l3FrwdQ[port][prio][frwdQHd];
			memcpy(l3Pkt, l3FrwdPkt, (sizeof(L3Hdr) + sizeof(L4Hdr)));
			const uint8_t dstSubnet = l3Pkt->hdr.dst >> 8;
			const uint8_t gateway = l3RouteTable[dstSubnet];
			*l2Addr = (gateway) ? gateway : l3Pkt->hdr.dst; /* if dst subnet is distant it will have gateway in route table */
			// TODO MST pass ?
			*xferMst = passMstFrwd(port, prio);
			return true;
		}

		
#endif
		/* Check Broadcast Stream */
		const l3BrdCstStrm_t* brdcstStream = &l3BrdcstStrms[port][prio];
		/* TODO should we care about tx order prempt or its ok for broadcast msgs to have higher priority over
		unicast? */
		if (l4TxBrdcastStrmEmpty(&brdcstStream->txPgPtr)) {
			brdcstStream = NULL;
		}

		bool txOrderPrempt = false;
		uint16_t dstPos = MAX_POS;

		/* for same prio, pending streams get priority after frwd */
		bool pndingStream = false;

		if (!brdcstStream || !l4TxBrdcstStrmPnding(&brdcstStream->txMsgHdr)) {
			/* check for premting by tx order */
			for (; pos < MAX_POS; pos++) {
				const uint8_t dstSubnet = (l3AddrTblPrio[pos][L3_DST_ADDR_IDX] & 0xFF00) >> 8;
				const uint8_t gatewaySubnet = (l3RouteTable[dstSubnet] & 0xFF00) >> 8;

				if (((dstSubnet == portSubnet) || (gatewaySubnet == portSubnet)) &&
					getL4Pkt(&l3Pkt->l4Pkt, pos, prio, l3Hdr->prio)) { // prempt by txOrder (Automatically prempts with pending stream as its txOrder will be lower)
					
					if (l4StrmPnding(dstPos, prio)) {
						dstPos = pos;
						break;
					}

					/* check to preemt broadcast stream with tx order */
					if (brdcstStream) {
						if (l4GetStrmTxOrder(pos, prio) > brdcstStream->txOrder) {
							continue; // dont select this stream
						}
						else {
							txOrderPrempt = true;
							brdcstStream = NULL;
						}
					}
					else if (dstPos != MAX_POS) {
						txOrderPrempt = true;
					}
					dstPos = pos;
				}
			}
		} else {
			pndingStream = true;
		}

		if (brdcstStream || dstPos != MAX_POS) { // send highest priority in tx order
			if (prio >= LOW_PRIO_IDX) {
				if (!pndingStream) { /* check to prempt with a downstream pending prio stream */
					brdcstStream = l3premptLowPrioPending(port, &l3Pkt->l4Pkt, &dstPos, &prio);
				}
				*xferMst = passMstStream(port, prio, dstPos, brdcstStream);
			}
			else if (prio < LOW_PRIO_IDX) { 
				// check if end of all prio streams before passing mst
				*xferMst = !txOrderPrempt && passMstStream(port, prio, dstPos, brdcstStream);
			}

			if (brdcstStream) {
				*l2Addr = setL3HdrBrdcst(l3Pkt, port, prio, dstPos);
			} else {
				*l2Addr = setL3HdrUnicst(l3Pkt, port, prio, dstPos);
			}
			return true;
		}
	}

	return false;
	//return false;
}

uint8_t getL3PktFrag(L3Pkt* const l3Pkt, uint8_t** ptr, uint8_t idx, uint8_t * txHd, uint8_t* txHdOfst, uint8_t txLen, uint8_t port) {
	L3Hdr* l3Hdr = &l3Pkt->hdr;

#if MAX_PORT > 1
	if (l3TxfrwdPkt(l3Hdr, port)) {
		const uint8_t frwdQHd = l3FrwdQHead[port][l3Hdr->prio];
		PgPtr_t * frwdPgPtr = &l3FrwdPgPtrQ[port][l3Hdr->prio][frwdQHd];
		return getL3FrwdPktFrag(frwdPgPtr, ptr, idx, txHd, txHdOfst, txLen);
	}
#endif

	if (l3TxBrdcstPkt(l3Hdr)) {
		l3BrdCstStrm_t* const brdcstStream = &l3BrdcstStrms[port][l3Hdr->prio];
		/* TODO change */
		return getL3BrdCstPktFrag(brdcstStream, ptr, idx, txHd, txHdOfst, txLen);
	}

	return getL4PktFrag(&l3Pkt->l4Pkt, ptr, idx, txHd, txHdOfst, txLen, l3Hdr->prio);
}

static inline bool l3CmtL4RxHd(L3Pkt *l3Pkt)
{
	L3Hdr *l3Hdr = &l3Pkt->hdr;
	const uint8_t prio = l3Hdr->prio;

	for (uint16_t pos = 0; pos < MAX_POS; pos++)
	{
		// check all possible pos addr
		for (int srcPort = 0; srcPort < MAX_PORT; srcPort++)
		{
			if (l3Hdr->src == l3AddrTblPrio[pos][srcPort])
			{
				l4CmtRxHd(&l3Pkt->l4Pkt, pos, prio);
				return true;
			}
		}
	}
	return false;
}

uint8_t getL3RxPktFrag(uint8_t port, L3Pkt *l3Pkt, uint8_t **ptr, uint8_t rxLen)
{
	L3Hdr *l3Hdr = &l3Pkt->hdr;

#if MAX_PORT > 1
	if (l3RxfrwdPkt(l3Hdr, port)) {
		return getL3RxFrwdPktFrag(&l3RxFrwdPktPgPtr[port], ptr, rxLen);
	}
#endif

	if (l4CmtRxPnding(&l3Pkt->l4Pkt)) { // hdr only message will no have identified stream yet ..
		if (!l3CmtL4RxHd(l3Pkt)) {
			return; // abort processing
		}
	}

	return getL4RxPktFrag(&l3Pkt->l4Pkt, ptr, rxLen, l3Hdr->prio);
}

bool l3CmtRxHd(L3Pkt *l3Pkt, const uint8_t port) { // called from L2
	L3Hdr *l3Hdr = &l3Pkt->hdr;

	if (l3Hdr->prio > MAX_PRIORITY)
	{ // check prio
		return false;
	}

#if MAX_PORT > 1
	if (l3RxfrwdPkt(l3Hdr, port)) {
		const uint8_t dstSubnet = l3Hdr->dst >> L3_ADDR_SUBNET_SHIFT;

		const uint8_t gatewaySubnet = l3RouteTable[dstSubnet] >> 8;

		for (uint8_t peerPort = 0; peerPort < MAX_PORT; peerPort++)
		{
			const uint16_t peerPortL3Addr = l3AddrTblPrio[myPos][peerPort];

			if (peerPort == port || !peerPortL3Addr)
			{
				continue;
			}

			// should not happen with the routing model but just incase
			if (peerPortL3Addr == l3Hdr->dst)
			{
				// update the dst for rx cmt so it dosnt think its forwarded pkt
				l3Hdr->dst = l3AddrTblPrio[myPos][port];
				return true;
			}

			const uint8_t peerPortSubnet = peerPortL3Addr >> 8;
			if ((dstSubnet == peerPortSubnet) ||
				(gatewaySubnet == peerPortSubnet))
			{ // forward on peer port
				l3Pkt->frwdPkt.dstPort = peerPort;
				return true;
			}
		}
		return false;
	}
#endif

	return true/*l3CmtL4RxHd(l3Pkt)*/;
}

bool l3RxCmplt(uint8_t rxIdx) {
	if (rxIdx <= sizeof(L3Hdr)) { // only recieved hdr or partial hdr
		/*
			if we accept l3 hdr only message TODO? commit l3 hd
		*/
		return false;
	}
	return true;
}

void l3CmtRx(L3Pkt * const l3Pkt, const uint8_t port)
{
	// check if message if for this device
	const L3Hdr *l3Hdr = &l3Pkt->hdr;

#if MAX_PORT > 1
	if (!l3Hdr->ttl) {
		return; // drop packet
	}
	
	if (l3RxfrwdPkt(l3Hdr, port)) {
		PgPtr_t *frwdQPgPtr = l3PushFrwdPkt(l3Pkt, l3Pkt->frwdPkt.dstPort);
		if (frwdQPgPtr) {
			/* copy pg ptr */
			memcpy(frwdQPgPtr, &l3RxFrwdPktPgPtr[port], sizeof(PgPtr_t));
			pgPtrInit(&l3RxFrwdPktPgPtr[port]);
		} else {
			/* Todo do we let the sender know  that frwd queue is full?*/
			freePgPtr(&l3RxFrwdPktPgPtr[port]);
		}
		return;
	}
#endif

	if (l4CmtRxPnding(&l3Pkt->l4Pkt)) { // hdr only message will no have identified stream yet ..
		if (!l3CmtL4RxHd(l3Pkt)) {
			return; // abort processing
		}
	}

#if MAX_PORT > 1
	/* Check to brodcast */
	if (l3TxBrdcstPkt(l3Hdr)) {
		l3BrdCst(port, l3Pkt);
	}
#endif

	l4CmtRx(&l3Pkt->l4Pkt, l3Hdr->prio);
}
