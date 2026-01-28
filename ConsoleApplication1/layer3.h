#ifndef L3_NETWORK
#define L3_NETWORK
#include "global.h"
#include "layer4.h"
//#include "layer2.h"



#define MAX_SUBNET 255

typedef struct __attribute__((packed)) {
	uint16_t src;
	uint16_t dst;
	uint8_t ttl;
	uint8_t prio;
} L3Hdr;

typedef struct __attribute__((packed)) {
	L3Hdr hdr;
	L4Pkt l4Pkt;
} L3Pkt;

void l3Init();

// return L2 addr 
// xferMst pass MST
bool getl3Pkt(L3Pkt* l3pkt, bool* xferMst, uint8_t* addr, uint8_t port);

void l3TxCmplt(L3Pkt* l3pkt);

bool l3Ack(L3Pkt* l3Pkt);

void getL3PktFrag(L3Pkt* l3Pkt, uint8_t** ptr, uint8_t* len, uint8_t* txHd, uint8_t* txHdOfst, uint8_t txLen);

#endif