//#include "layer2.h"
#include "layer3.h"
//#include "layer4.h"
#include "network.h"

uint16_t l3AddrTable[MAX_POS];
uint16_t l3RouteTable[MAX_SUBNET];

void l3Init() {
	//memset(port_ip, 0, sizeof port_ip);
   // setPortAddr();
}

bool l3Ack(L3Pkt* l3Pkt) {
	L3Hdr* l3Hdr = &l3Pkt->hdr;
	return l4Ack(&l3Pkt->l4Pkt, l3Hdr->prio, l3Hdr->dst);
}

void l3TxCmplt(L3Pkt* l3Pkt) {
	l4TxCmplt(&l3Pkt->l4Pkt);
}

uint8_t getL3PktHd(L3Pkt * l3Pkt, uint8_t * ofst) {
	return getL4PktHd(&l3Pkt->l4Pkt, ofst);
}

uint8_t setL3Hdr(L3Pkt * l3Pkt, uint8_t port) {
	L3Hdr* l3Hdr = &l3Pkt->hdr;
	l3Hdr->src = port_addr[port];
	l3Hdr->ttl = 1;
	l3Hdr->prio = l3Pkt->l4Pkt.psIdx;
	l3Hdr->dst = l3AddrTable[l3Pkt->l4Pkt.sIdx];
	setL4Hdr(&l3Pkt->l4Pkt);
	const uint8_t dstSubnet = (l3Hdr->dst & 0xFF00) >> 8;
	return l3RouteTable[dstSubnet]; // return l2Addr
}

bool getl3Pkt(uint8_t port, L3Pkt* l3Pkt, bool* xferMst) {
	*xferMst = false;
	const uint8_t portSubnet = ((port_addr[port] & 0xFF00) >> 8);
	bool ret = false;

	for (uint8_t prio = 0; prio < MAX_PRIORITY; prio++) {
		for (uint16_t pos = 0; pos < MAX_POS; pos++) {
			const uint8_t dstSubnet = (l3AddrTable[pos] & 0xFF00) >> 8;
			const uint8_t gatewaySubnet = (l3RouteTable[dstSubnet] & 0xFF00) >> 8;
			
			if (gatewaySubnet == portSubnet) {
				ret |= getL4Pkt(&l3Pkt->l4Pkt, pos, prio);
			}
		}

		if (ret) { // send highest priority in tx order
			break;
		}
	}

	return ret;

#if 0
	for (uint8_t prio = 0; prio < MAX_PRIORITY; prio++) {
		uint16_t dst;
		uint8_t portSubnet = ((port_addr[port] & 0xFF00) >> 8);
		if (getL4Pkt(&l3Pkt->l4Pkt, portSubnet, prio, &dst)) {
			L3Hdr* l3Hdr = &l3Pkt->hdr;
			l3Hdr->dst = dst;
			l3Hdr->src = port_addr[port];
			l3Hdr->ttl = 1; // TODO
			l3Hdr->prio = prio;
			return true;
		}
	}
#endif
	//return false;
}

void getL3PktFrag(L3Pkt* l3Pkt, uint8_t** ptr, uint8_t* len, uint8_t * txHd, uint8_t* txHdOfst, uint8_t txLen) {

	// check here if its a forwarded pkt ?
	getL4PktFrag(&l3Pkt->l4Pkt, ptr, len, txHd, txHdOfst, txLen);
}