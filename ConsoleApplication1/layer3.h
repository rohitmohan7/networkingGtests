#ifndef L3_NETWORK
#define L3_NETWORK
#include "global.h"
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
	uint8_t  head_page;
	uint8_t  tail_page;
	uint8_t  head_off;       /* 0..UNIT-1 */
	uint8_t  tail_used;
} L3Pkt;


void l3Init();

// return L2 addr 
// xferMst pass MST
struct L2Pkt;   // <-- adds the typedef name
bool getl3Pkt(struct L2Pkt* l2pkt, bool* xferMst, uint8_t* addr, uint8_t port);
#endif