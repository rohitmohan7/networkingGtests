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

uint8_t setL3Hdr(L3Pkt * l3Pkt, uint8_t port, uint8_t prio, uint16_t pos) {
	L3Hdr* l3Hdr = &l3Pkt->hdr;
	l3Hdr->src = port_addr[port];
	l3Hdr->ttl = 1;
	l3Hdr->prio = prio;
	l3Hdr->dst = l3AddrTable[pos];
	setL4Hdr(&l3Pkt->l4Pkt);
	const uint8_t dstSubnet = (l3Hdr->dst & 0xFF00) >> 8;
	return l3RouteTable[dstSubnet]; // return l2Addr
}

bool getl3Pkt(uint8_t port, L3Pkt* l3Pkt, bool* xferMst, uint8_t * l2Addr) {
	*xferMst = false;
	const uint8_t portSubnet = ((port_addr[port] & 0xFF00) >> 8);

	for (uint8_t prio = 0; prio < MAX_PRIORITY; prio++) {
		uint16_t dstPos = MAX_POS;
		for (uint16_t pos = 0; pos < MAX_POS; pos++) {
			const uint8_t dstSubnet = (l3AddrTable[pos] & 0xFF00) >> 8;
			const uint8_t gatewaySubnet = (l3RouteTable[dstSubnet] & 0xFF00) >> 8;
			
			if (gatewaySubnet == portSubnet && getL4Pkt(&l3Pkt->l4Pkt, pos, prio)) {
				dstPos = pos;
			}
		}

		if (dstPos != MAX_POS) { // send highest priority in tx order
			*l2Addr = setL3Hdr(l3Pkt, port, prio, dstPos);
			return true;
		}
	}

	return false;
	//return false;
}

void getL3PktFrag(L3Pkt* l3Pkt, uint8_t** ptr, uint8_t* len, uint8_t * txHd, uint8_t* txHdOfst, uint8_t txLen) {

	// check here if its a forwarded pkt ?
	getL4PktFrag(&l3Pkt->l4Pkt, ptr, len, txHd, txHdOfst, txLen);
}