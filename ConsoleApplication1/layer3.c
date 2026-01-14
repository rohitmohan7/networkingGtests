//#include "layer2.h"
#include "layer3.h"
//#include "layer4.h"
#include "network.h"


void l3Init() {
	//memset(port_ip, 0, sizeof port_ip);
   // setPortAddr();
}

bool l3Ack(L3Pkt* l3Pkt) {
	L3Hdr* l3Hdr = &l3Pkt->hdr;
	return l4Ack(&l3Pkt->l4Pkt, l3Hdr->prio, l3Hdr->dst);
}

bool getl3Pkt(L3Pkt* l3Pkt, bool* xferMst, uint8_t* addr, uint8_t port) {
	bool checkPrioPending = false;
	*xferMst = false;
	for (uint8_t prio = 0; prio < MAX_PRIORITY; prio++) {
		uint16_t dst;
		uint8_t portSubnet = ((port_addr[port] & 0xFF00) >> 8);
		if (getL4Pkt(&l3Pkt->l4Pkt, portSubnet, prio, &dst, addr)) {
			L3Hdr* l3Hdr = &l3Pkt->hdr;
			l3Hdr->dst = dst;
			l3Hdr->src = port_addr[port];
			l3Hdr->ttl = 1; // TODO
			l3Hdr->prio = prio;
			return true;
		}
	}
	return false;
	//return checkPrioPending;
}