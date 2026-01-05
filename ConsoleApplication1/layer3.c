//#include "layer2.h"
#include "layer3.h"
//#include "layer4.h"
#include "network.h"



uint16_t pos_addr_table[MAX_POS];
uint16_t route_table[MAX_SUBNET];

void l3Init() {
	memset(pos_addr_table, 0, sizeof pos_addr_table);
	memset(route_table, 0, sizeof route_table);
	//memset(port_ip, 0, sizeof port_ip);
   // setPortAddr();
}

void l3Ack(L3Pkt* l3Pkt) {
	L3Hdr* l3Hdr = &l3Pkt->hdr;
	l4Ack(&l3Pkt->l4Pkt, l3Hdr->prio, l3Hdr->dst);
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
#if 0
			if (((s->gateway & 0xFF00) == (port_addr[port] & 0xFF00)) 
				&& ps->head_page != INVALID_PAGE) {

				//if (checkPrioPending) {
				//	*xferMst = false;
				//	return true;
				//}

				// set header
				L3Pkt* l3Pkt = &l2pkt->msg.pdu.l3pkt;
				L3Hdr* l3Hdr = &l3Pkt->hdr;
			
				l3Hdr->src = port_addr[port];
				l3Hdr->dst = s->dst;
				//l3hdr->ttl = TODO
				l3Hdr->prio = prio;
 
				uint8_t len = L3_FRAME_SIZE;
				l3Pkt->head_page = ps->head_page;
				l3Pkt->head_off = ps->head_off;

				uint8_t currHd = l3Pkt->head_page;

				while (len > 0) {
					if (currHd == ps->tail_page ||
						len < (UNIT)) { // reached end
						l3Pkt->tail_page = currHd;
						l3Pkt->tail_used = currHd == ps->tail_page? min(len, ps->tail_used): len;
						break;
					}

					currHd = g_next[currHd];
					len -= UNIT;
				}
				checkPrioPending = true;
#endif
	}
	return false;
	//return checkPrioPending;
}