#include "layer3.h"

uint16_t pos_addr_table[MAX_POS];
uint16_t route_table[MAX_SUBNET];

void l3Init() {
	memset(pos_addr_table, 0, sizeof pos_addr_table);
	memset(route_table, 0, sizeof route_table);
	//memset(port_ip, 0, sizeof port_ip);
   // setPortAddr();
}

bool getl3Pkt(struct L2Pkt* l2pkt, bool* xferMst, uint8_t* addr, uint8_t port) {
	return false;
}