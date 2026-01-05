#ifndef L3_NETWORK
#define L3_NETWORK
#include "global.h"
#include "layer4.h"
//#include "layer2.h"



#define MAX_SUBNET 255

extern uint16_t pos_addr_table[MAX_POS];
extern uint16_t route_table[MAX_SUBNET];

typedef struct {
	uint16_t src;
	uint16_t dst;
	uint8_t ttl;
	uint8_t prio;
} L3Hdr;

typedef struct {
	L3Hdr hdr;
	L4Pkt l4Pkt;
} L3Pkt;

void l3Init();

// return L2 addr 
// xferMst pass MST
bool getl3Pkt(L3Pkt* l3pkt, bool* xferMst, uint8_t* addr, uint8_t port);
#endif