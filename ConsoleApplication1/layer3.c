#include "layer2.h"
#include "layer3.h"
#include "layer4.h"
#include "network.h"

#define L3_FRAME_SIZE (L2_FRAME_SIZE - sizeof(L3Hdr))

uint16_t pos_addr_table[MAX_POS];
uint16_t route_table[MAX_SUBNET];

void l3Init() {
	memset(pos_addr_table, 0, sizeof pos_addr_table);
	memset(route_table, 0, sizeof route_table);
	//memset(port_ip, 0, sizeof port_ip);
   // setPortAddr();
}

bool getl3Pkt(struct L2Pkt* l2pkt, bool* xferMst, uint8_t* addr, uint8_t port) {
	bool checkPrioPending = false;
	*xferMst = true;
	for (uint8_t prio = 0; prio < MAX_PRIORITY; prio++) {
		for (int pos = 0; pos < MAX_POS; pos++) {
			stream_t* s = &streams[pos];
			prio_stream_t * ps = &s->prio[prio];

			if (((s->gateway & 0xFF00) == (port_addr[port] & 0xFF00)) 
				&& ps->head_page != INVALID_PAGE) {

				if (checkPrioPending) {
					*xferMst = false;
					return true;
				}

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
					if (l3Pkt->head_page == ps->tail_page) {

					}
				}
				checkPrioPending = true;
			}
		}
	}
	return checkPrioPending;
}