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

L3Pkt l3FrwdQ[MAX_PRIORITY][MAX_FORWARD_QUEUE];

static uint8_t l3FrwdQHead[MAX_PRIORITY];
static uint8_t l3FrwdQTail[MAX_PRIORITY];
static uint8_t l3FrwdQCount[MAX_PRIORITY];

#define LOW_PRIO_IDX 1

static inline void l3FrwdQInit(void)
{
	memset(l3FrwdQ, 0, sizeof(l3FrwdQ));
	memset(l3FrwdQHead, 0, sizeof(l3FrwdQHead));
	memset(l3FrwdQTail, 0, sizeof(l3FrwdQTail));
	memset(l3FrwdQCount, 0, sizeof(l3FrwdQCount));
}

void l3Init(void) {
	l3FrwdQInit();
	//memset(port_ip, 0, sizeof port_ip);
   // setPortAddr();
}

void l3TxCmplt(L3Pkt* l3Pkt) {
	L3Hdr* l3Hdr = &l3Pkt->hdr;
	l4TxCmplt(&l3Pkt->pkt.l4Pkt, l3Hdr->prio);
}

bool getL3PktHd(L3Pkt *l3Pkt, uint8_t *hd, uint8_t *ofst)
{
	L3Hdr* l3Hdr = &l3Pkt->hdr;
	return getL4PktHd(&l3Pkt->pkt.l4Pkt, l3Hdr->prio, hd, ofst);
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

bool passMst(uint8_t prioIdx, uint16_t posIdx) {

	if ((prioIdx < LOW_PRIO_IDX && !l4StrmEmptyAftFrme(posIdx, prioIdx)) ||
		 (prioIdx >= LOW_PRIO_IDX && l4StrmPnding(posIdx, prioIdx))) {
		/* If current high prio stream is not empty after frame xmit OR if a low prio stream is pending 
		* dont pass MST token
		*/
		return false;
	}

	if (prioIdx >= LOW_PRIO_IDX && l4lstMsgFrm(posIdx, prioIdx)) {
		// if this the last frame of a low priority message pass the MST token
		return true;
	}

	if (posIdx + 1 >= MAX_POS) {
		posIdx = 0;
		prioIdx++; // check from next priority
	}
	else {
		posIdx++; // check from next pos 
	}

	uint16_t pos = posIdx;

	for (uint8_t prio = prioIdx; prio < MAX_PRIORITY; prio++) {
		for (; pos < MAX_POS; pos++) {
			if (prio >= LOW_PRIO_IDX) { // check for pending low prio
				if (l4StrmPnding(pos, prio)) {
					return false;
				}
			} else if (!l4StrmEmpty(pos, prio)) {
				return false;
			}
		}
		pos = 0;
	}
	return true;
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
				*xferMst = passMst(prio, dstPos);
			}
			else if (prio < LOW_PRIO_IDX) { 
				// check if end of all prio streams before passing mst
				*xferMst = !txOrderPrempt && passMst(prio, dstPos);
			}
			*l2Addr = setL3Hdr(l3Pkt, port, prio, dstPos);
			return true;
		}
	}

	return false;
	//return false;
}

uint8_t getL3PktFrag(L3Pkt* l3Pkt, uint8_t** ptr, uint8_t idx, uint8_t * txHd, uint8_t* txHdOfst, uint8_t txLen) {
	L3Hdr* l3Hdr = &l3Pkt->hdr;
	// check here if its a forwarded pkt ?
	return getL4PktFrag(&l3Pkt->pkt.l4Pkt, ptr, idx, txHd, txHdOfst, txLen, l3Hdr->prio);
}

uint8_t getL3RxPktFrag(uint8_t port, L3Pkt *l3Pkt, uint8_t **ptr, uint8_t idx, uint8_t rxLen)
{
	L3Hdr *l3Hdr = &l3Pkt->hdr;
	if (l3Hdr->dst == l3AddrTblPrio[myPos][port]) {
		return getL4RxPktFrag(&l3Pkt->pkt.l4Pkt, ptr, idx, rxLen, l3Hdr->prio);
	}
	else { // TODO forward
		return false;
	}
}

static inline bool l3CmtL4RxHd(L3Pkt* l3Pkt) {
	L3Hdr* l3Hdr = &l3Pkt->hdr;
	for (uint16_t pos = 0; pos < MAX_POS; pos++) {
		// check all possible pos addr
		for (int srcPort = 0; srcPort < MAX_PORT; srcPort++) {
			if (l3Hdr->src == l3AddrTblPrio[pos][srcPort])
			{
				return l4CmtRxHd(&l3Pkt->pkt.l4Pkt, pos, l3Hdr->prio);
			}
		}
	}
	return false;
}

bool l3CmtRxHd(L3Pkt *l3Pkt, const uint8_t port) {
	L3Hdr *l3Hdr = &l3Pkt->hdr;
	const uint8_t prio = l3Hdr->prio;

	if (prio > MAX_PRIORITY) { // check prio
		return false;
	}

	if (l3Hdr->dst == l3AddrTblPrio[myPos][port]) {
		return l3CmtL4RxHd(l3Pkt);
	} else { // TODO forward
		const uint8_t dstSubnet = l3Hdr->dst >> L3_ADDR_SUBNET_SHIFT;
		if (l3RouteTable[dstSubnet]) { // check if there is a gateway
			// 
			if (l3FrwdQCount[prio] >= MAX_FORWARD_QUEUE) {
				// TODO do we tx L3 forward Queue full?

				return false; /* full */
			}

			l3Pkt->pkt.frwdPkt = &l3FrwdQ[prio][l3FrwdQTail[prio]];
			l3FrwdQTail[prio] = (uint8_t)((l3FrwdQTail[prio] + 1u) % MAX_FORWARD_QUEUE);
			return true;
		}
		else {
			// l3 tx unreachable ..
		}


	}
	
	return false;
}

void l3CmtRx(L3Pkt *l3Pkt, const uint8_t port)
{
	// check if message if for this device
	const L3Hdr *l3Hdr = &l3Pkt->hdr;
	if (!l3Hdr->ttl) {
		return; // drop packet
	}

	if (l3Hdr->dst == l3AddrTblPrio[myPos][port])
	{
		if (l4CmtRxPnding(&l3Pkt->pkt.l4Pkt)) { // hdr only message will no have identified stream yet ..
			if (!l3CmtL4RxHd(l3Pkt)) {
				return;
			}
		}

		l4CmtRx(&l3Pkt->pkt.l4Pkt, l3Hdr->prio);
	} else { // TODO forward packet
		
	}
}
