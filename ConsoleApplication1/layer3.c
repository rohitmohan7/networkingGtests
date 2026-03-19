//#include "layer2.h"
#include "layer3.h"
//#include "layer4.h"
#include "network.h"

#define L3_DST_ADDR_IDX 0
#define L3_ADDR_SUBNET_SHIFT 8

uint16_t l3AddrTblPrio[MAX_POS][MAX_PORT]; // pos addresses ordered by tx priority
uint16_t l3RouteTable[MAX_SUBNET]; // next best gateway for subnet
uint8_t l3BcastInSubnetForSrcPort[MAX_POS][MAX_PORT];
uint8_t l3RouteHops[MAX_SUBNET]; // l3 hop table for my pos

#define MAX_FORWARD_QUEUE 4 // power of 2
#define MAX_FORWARD_QUEUE_MASK (MAX_FORWARD_QUEUE - 1u)

L3Pkt l3FrwdQ[MAX_PORT][MAX_PRIORITY][MAX_FORWARD_QUEUE];

static uint8_t l3FrwdQHead[MAX_PORT][MAX_PRIORITY];
static uint8_t l3FrwdQTail[MAX_PORT][MAX_PRIORITY];
static uint8_t l3FrwdQCount[MAX_PORT][MAX_PRIORITY];

#define LOW_PRIO_IDX 1

static inline void l3FrwdQInit(void)
{
	memset(l3FrwdQ, 0, sizeof(l3FrwdQ));
	
	// invalidate the page ptrs
	for (uint8_t port = 0; port < MAX_PORT; port++) {
		for (uint8_t prio = 0; prio < MAX_PRIORITY; prio++) {
			for (uint8_t queueIdx = 0; queueIdx < MAX_FORWARD_QUEUE; queueIdx++) {
				pgPtrInit(&l3FrwdQ[port][prio][queueIdx].pkt.frwdPkt);
			}
		}
	}
	
	memset(l3FrwdQHead, 0, sizeof(l3FrwdQHead));
	memset(l3FrwdQTail, 0, sizeof(l3FrwdQTail));
	memset(l3FrwdQCount, 0, sizeof(l3FrwdQCount));
}

void l3Init(void) {
	l3FrwdQInit();
	//memset(port_ip, 0, sizeof port_ip);
   // setPortAddr();
}

static inline void l3FwdTxCmplt(L3Pkt *l3Pkt, const uint8_t port)
{
	// free the allocated pages
	freePgPtr(l3Pkt->pkt.frwdPktPtr);
	const uint8_t prio = l3Pkt->hdr.prio;
		// free the forward queue
	l3FrwdQHead[port][prio] = (uint8_t)((l3FrwdQHead[port][prio] + 1u) % MAX_FORWARD_QUEUE);
	l3FrwdQCount[port][prio]--;
}

void l3TxCmplt(L3Pkt* l3Pkt, const uint8_t port) {
	L3Hdr* l3Hdr = &l3Pkt->hdr;

	if (l3Hdr->src == l3AddrTblPrio[myPos][port]) {
		l4TxCmplt(&l3Pkt->pkt.l4Pkt, l3Hdr->prio);
	} else {
		l3FwdTxCmplt(l3Pkt, port);
	}
}

static inline bool getL3FrwdPktHd(L3Pkt *l3Pkt, uint8_t *hd, uint8_t *ofst)
{
	if (l3Pkt->pkt.frwdPktPtr)
	{ // could be message with just a hdr
		*ofst = l3Pkt->pkt.frwdPktPtr->hdOfst;
		*hd = l3Pkt->pkt.frwdPktPtr->hdPg;
		return false;
	}
	return true;
}

uint8_t l3GetTxPktHdrSize(L3Pkt *l3Pkt, uint8_t port) {
	L3Hdr *l3Hdr = &l3Pkt->hdr;
	if (l3Hdr->src == l3AddrTblPrio[myPos][port]) {
		return sizeof(L3Hdr) + sizeof(L4Hdr);
	} else {
		return sizeof(L3Hdr);
	}
}

uint8_t l3GetRxPktHdrSize(L3Pkt *l3Pkt, uint8_t port)
{
	L3Hdr *l3Hdr = &l3Pkt->hdr;
	if (l3Hdr->dst == l3AddrTblPrio[myPos][port]) {
		return sizeof(L3Hdr) + sizeof(L4Hdr);
	} else {
		return sizeof(L3Hdr); // forward packet
	}
}

bool getL3PktHd(L3Pkt *l3Pkt, uint8_t *hd, uint8_t *ofst, uint8_t port)
{
	L3Hdr* l3Hdr = &l3Pkt->hdr;
	if (l3Hdr->src == l3AddrTblPrio[myPos][port]) {
		return getL4PktHd(&l3Pkt->pkt.l4Pkt, l3Hdr->prio, hd, ofst);
	} else {
		return getL3FrwdPktHd(l3Pkt, hd, ofst);
	}
}

uint8_t setL3Hdr(L3Pkt * l3Pkt, uint8_t port, uint8_t prio, uint16_t pos) {
	L3Hdr* l3Hdr = &l3Pkt->hdr;
	l3Hdr->src = l3AddrTblPrio[myPos][port];
	l3Hdr->ttl = 1;
	l3Hdr->prio = prio;
	l3Hdr->dst = l3AddrTblPrio[pos][L3_DST_ADDR_IDX];
	setL4Hdr(&l3Pkt->pkt.l4Pkt, prio);
	const uint8_t dstSubnet = (l3Hdr->dst & 0xFF00) >> 8;
	const uint8_t portSubnet = ((l3Hdr->src & 0xFF00) >> 8);
	return ((dstSubnet == portSubnet)? l3Hdr->dst: l3RouteTable[dstSubnet]) & 0x00FF; // return l2Addr
}

bool passMst(uint8_t port, uint8_t prioIdx, uint16_t pos) {

	for (uint8_t prio = prioIdx; prio < MAX_PRIORITY; prio++)
	{
		/* Dont check frwd stream for current prioIdx (already checked!)*/
		if ((prio < LOW_PRIO_IDX) && (prio != prioIdx) && l3FrwdQCount[port][prio])
		{
			return false;
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

bool passMstFrwd(const uint8_t port, const uint8_t prioIdx)
{
	if (prioIdx < LOW_PRIO_IDX &&
		l3FrwdQCount[port][prioIdx] > 1) // more high prio frwd pkts waiting
	{
		return false;
	}

	return passMst(port, prioIdx, 0); // check from pos 0 of this prio if there is a pending high prio stream or frwd pkt
}

bool passMstStream(const uint8_t port, uint8_t prioIdx, uint16_t posIdx) {

	if (prioIdx < LOW_PRIO_IDX)
	{
		if (!l4StrmEmptyAftFrme(posIdx, prioIdx)) {
			return false;
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
	} else {
		return l4lstMsgFrm(posIdx, prioIdx); // if this the last frame of a low priority message pass the MST token
	}	
}

void l3premptLowPrioPending(L3Pkt* l3Pkt, uint16_t* posIdx, uint8_t* prioIdx) {

	if (l4StrmPnding(*posIdx, *prioIdx)) { // aready current stream is pending
		return;
	}

	prioIdx++; // txOrder premting would have already selected a pending stream
	L3Hdr* l3Hdr = &l3Pkt->hdr;

	for (uint8_t prio = prioIdx; prio < MAX_PRIORITY; prio++) {
		for (int pos = 0; pos < MAX_POS; pos++) {
			if (l4StrmPnding(pos, prio)) { // prempt with this stream
				if (getL4Pkt(&l3Pkt->pkt.l4Pkt, pos, prio, l3Hdr->prio)) {
					*posIdx = pos;
					*prioIdx = prio;
					return;
				} // this is an error condition should not get here
			}
		}
	}
}

bool getl3Pkt(uint8_t port, L3Pkt* l3Pkt, bool* xferMst, uint8_t * l2Addr) {
	*xferMst = false;
	const uint8_t portSubnet = ((l3AddrTblPrio[myPos][port] & 0xFF00) >> 8);
	L3Hdr* l3Hdr = &l3Pkt->hdr;

	for (uint8_t prio = 0; prio < MAX_PRIORITY; prio++) {
		
		// check forward packet first
		if (l3FrwdQCount[port][prio])
		{
			/* Copy the header */
			L3Pkt *l3FrwdPkt = &l3FrwdQ[port][prio][l3FrwdQHead[port][prio]];
			memcpy(&l3Pkt->hdr, &l3FrwdPkt->hdr, sizeof(L3Hdr));
			l3Pkt->pkt.frwdPktPtr = &l3FrwdPkt->pkt.frwdPkt;
			*l2Addr = l3Pkt->hdr.dst;
			// TODO MST pass ?
			*xferMst = passMstFrwd(port, prio);
			return true;
		}

		uint16_t dstPos = MAX_POS;
		bool txOrderPrempt = false;
		
		for (uint16_t pos = 0; pos < MAX_POS; pos++) {
			const uint8_t dstSubnet = (l3AddrTblPrio[pos][L3_DST_ADDR_IDX] & 0xFF00) >> 8;
			const uint8_t gatewaySubnet = (l3RouteTable[dstSubnet] & 0xFF00) >> 8;
			
			if (((dstSubnet == portSubnet) || (gatewaySubnet == portSubnet)) &&
					getL4Pkt(&l3Pkt->pkt.l4Pkt, pos, prio, l3Hdr->prio)) { // prempt by txOrder (Automatically prempts with pending stream as its txOrder will be lower)
				if (dstPos != MAX_POS) {
					txOrderPrempt = true;
				}
				dstPos = pos;
			}
		}

		if (dstPos != MAX_POS) { // send highest priority in tx order
			if (prio >= LOW_PRIO_IDX) {
				l3premptLowPrioPending(&l3Pkt->pkt.l4Pkt, &dstPos, &prio);
				*xferMst = passMstStream(port, prio, dstPos);
			}
			else if (prio < LOW_PRIO_IDX) { 
				// check if end of all prio streams before passing mst
				*xferMst = !txOrderPrempt && passMstStream(port, prio, dstPos);
			}
			*l2Addr = setL3Hdr(l3Pkt, port, prio, dstPos);
			return true;
		}
	}

	return false;
	//return false;
}

static inline uint8_t getL3FrwdPktFrag(L3Pkt * const l3Pkt, uint8_t **ptr, uint8_t idx, uint8_t *txHd, uint8_t *txHdOfst, uint8_t txLen)
{
	const uint16_t base = pageOff(*txHd) + (uint16_t)(*txHdOfst);

	*ptr = &g_pool[base];

	// cap tx len
	uint8_t len = (*txHd == l3Pkt->pkt.frwdPktPtr->tlPg ? (l3Pkt->pkt.frwdPktPtr->tlUsd - *txHdOfst) : (UNIT - (*txHdOfst)));

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

	return len;
}

uint8_t getL3PktFrag(L3Pkt* const l3Pkt, uint8_t** ptr, uint8_t idx, uint8_t * txHd, uint8_t* txHdOfst, uint8_t txLen, uint8_t port) {
	L3Hdr* l3Hdr = &l3Pkt->hdr;
	// check here if its a forwarded pkt ?
	if (l3Hdr->src == l3AddrTblPrio[myPos][port])
	{
		return getL4PktFrag(&l3Pkt->pkt.l4Pkt, ptr, idx, txHd, txHdOfst, txLen, l3Hdr->prio);
	} else {
		return getL3FrwdPktFrag(l3Pkt, ptr, idx, txHd, txHdOfst, txLen);
	}
}

static inline uint8_t getL3RxFrwdPktFrag(PgPtr_t * const frwdPktPtr, uint8_t **ptr, uint8_t rxLen)
{
	uint8_t len;
	*ptr = getPgPtr(frwdPktPtr, &len, rxLen);
	return len;
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
				l4CmtRxHd(&l3Pkt->pkt.l4Pkt, pos, prio);
				return true;
			}
		}
	}
	return false;
}

uint8_t getL3RxPktFrag(uint8_t port, L3Pkt *l3Pkt, uint8_t **ptr, uint8_t rxLen)
{
	L3Hdr *l3Hdr = &l3Pkt->hdr;
	if (l3Hdr->dst == l3AddrTblPrio[myPos][port]) {
		if (l4CmtRxPnding(&l3Pkt->pkt.l4Pkt)) {
			if (!l3CmtL4RxHd(l3Pkt)){
				return 0;
			}
		}

		return getL4RxPktFrag(&l3Pkt->pkt.l4Pkt, ptr, rxLen, l3Hdr->prio);
	}
	else {
		return getL3RxFrwdPktFrag(l3Pkt->pkt.frwdPktPtr, ptr, rxLen);
	}
}

static inline L3Pkt * l3PushFrwdPkt(L3Pkt *const l3Pkt, const uint8_t port)
{
	const uint8_t prio = l3Pkt->hdr.prio;
	
	if ((prio > MAX_PRIORITY) ||
		(l3FrwdQCount[port][prio] >= MAX_FORWARD_QUEUE))
	{
		// TODO do we tx L3 forward Queue full or prio error?
		return NULL; /* full */
	}

	L3Pkt *l3FrwdPkt = &l3FrwdQ[port][prio][l3FrwdQTail[port][prio]];

	/* Copy the header */
	memcpy(&l3FrwdPkt->hdr, &l3Pkt->hdr, sizeof(L3Hdr));

	l3FrwdQTail[port][prio] = (uint8_t)((l3FrwdQTail[port][prio] + 1u) % MAX_FORWARD_QUEUE);
	l3FrwdQCount[port][prio]++;
	return l3FrwdPkt;
}

static inline bool l3PushFrwdPktUnicst(L3Pkt *const l3Pkt, const uint8_t port)
{
	L3Pkt *l3FrwdPkt = l3PushFrwdPkt(l3Pkt, port);
	if (l3FrwdPkt)
	{
		l3Pkt->pkt.frwdPktPtr = &l3FrwdPkt->pkt.frwdPkt;
		return true;
	}
	return false;
}

static inline bool l3PushFrwdPktBrdcst(L3Pkt *const l3Pkt, const uint8_t port)
{
	L3Pkt *l3FrwdPkt = l3PushFrwdPkt(l3Pkt, port);
	if (l3FrwdPkt)
	{
		/* need to copy local packets */
		return true;
	}
	return false;
}

bool l3CmtRxHd(L3Pkt *l3Pkt, const uint8_t port) { // called from L2
	L3Hdr *l3Hdr = &l3Pkt->hdr;

	if (l3Hdr->prio > MAX_PRIORITY)
	{ // check prio
		return false;
	}

	if (l3Hdr->dst &&
			 l3Hdr->dst != l3AddrTblPrio[myPos][port]) {
		const uint8_t dstSubnet = l3Hdr->dst >> L3_ADDR_SUBNET_SHIFT;
		
		const uint8_t gatewaySubnet = l3RouteTable[dstSubnet] >> 8;
		
		for (uint8_t peerPort = 0; peerPort < MAX_PORT; peerPort++)
		{
			const uint16_t peerPortL3Addr = l3AddrTblPrio[myPos][peerPort];

			if (peerPort == port || !peerPortL3Addr) {
				continue;
			}

			// should not happen with the routing model but just incase
			if (peerPortL3Addr == l3Hdr->dst) {
				// update the dst for rx cmt so it dosnt think its forwarded pkt
				l3Hdr->dst = l3AddrTblPrio[myPos][port];
				return true;
			}

			const uint8_t peerPortSubnet = peerPortL3Addr >> 8;
			if ((dstSubnet == peerPortSubnet) ||
				(gatewaySubnet == peerPortSubnet))
			{ // forward on peer port
				return l3PushFrwdPktUnicst(l3Pkt, peerPort);
			}
		}
		
#if 0
		if (l3RouteTable[dstSubnet]) { // check if there is a gateway
			//
			return l3PushFrwdPkt(l3Pkt, port);
		}
		else {
			// check for local forwarding
			for (uint8_t peerPort = 0; peerPort < MAX_PORT; peerPort++)
			{
				if (peerPort == port) {
					continue;
				}
				
				const uint16_t peerPortL3Addr = l3AddrTblPrio[myPos][peerPort];
	
				// should not happen with the routing model but just incase
				if (peerPortL3Addr == l3Hdr->dst) {
					// update the dst for rx cmt so it dosnt think its forwarded pkt
					l3Hdr->dst = l3AddrTblPrio[myPos][port];
					return true;
				}

				const uint8_t peerPortSubnet = peerPortL3Addr >> 8;
				if (dstSubnet == peerPortSubnet) { // forward on peer port
					return l3PushFrwdPkt(l3Pkt, peerPort);
				}
			}
			// l3 tx unreachable ..
		}
#endif
	} else {
		return true; // wait for l4 hdr commmit
	}

	return false;
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

static inline void l3BrdCst(const uint8_t port, L3Pkt * const l3Pkt)
{
	const uint8_t portSubnet = l3AddrTblPrio[myPos][port] >> 8;
	for (int peerPort = 0; peerPort < MAX_PORT; peerPort++)
	{
		if (peerPort == port)
		{
			continue;
		}
		
		// check brdcast table
		if (l3BcastInSubnetForSrcPort[l3Pkt->pkt.l4Pkt.pos][peerPort] == portSubnet)
		{
			if (l3PushFrwdPktBrdcst(l3Pkt, peerPort))
			{
			}
		}
	}
}

void l3CmtRx(L3Pkt * const l3Pkt, const uint8_t port)
{
	// check if message if for this device
	const L3Hdr *l3Hdr = &l3Pkt->hdr;
	if (!l3Hdr->ttl) {
		return; // drop packet
	}

	// TODO will there be L3 hdr ONLY messages pending l3 hd commit pr reject msg as incomplete?

	if (!l3Hdr->dst || 
			l3Hdr->dst == l3AddrTblPrio[myPos][port])
	{
		if (l4CmtRxPnding(&l3Pkt->pkt.l4Pkt)) { // hdr only message will no have identified stream yet ..
			if (!l3CmtL4RxHd(l3Pkt)) {
				return; // abort processing
			}
		}

		l4CmtRx(&l3Pkt->pkt.l4Pkt, l3Hdr->prio);

		/* Check to brodcast */
		if (!l3Hdr->dst)
		{
			l3BrdCst(port, l3Pkt);
		}
	} else { // TODO forward packet
		
	}
}
