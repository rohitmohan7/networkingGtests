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
	for (uint8_t prio = 0; prio < MAX_PRIORITY; prio++) {
		for (int pos = 0; pos < MAX_POS; pos++) {
			stream_t* s = &streams[pos];
			prio_stream_t * ps = &s->prio[prio];

			if (((s->gateway & 0xFF00) == (port_addr[port] & 0xFF00)) 
				&& ps->head_page != INVALID_PAGE) {
				// set header
				L3Hdr* l3hdr = &l2pkt->msg.pdu.l3pkt.hdr;

				uint8_t len = L3_FRAME_SIZE;
			}
		}
	}
	return false;
}