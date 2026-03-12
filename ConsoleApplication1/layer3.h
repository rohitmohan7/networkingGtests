#ifndef L3_NETWORK
#define L3_NETWORK
#include "layer4.h"
//#include "layer2.h"

#define MAX_SUBNET 256

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

extern uint16_t l3AddrTable[MAX_POS];
extern uint16_t l3RouteTable[MAX_SUBNET]; // gateway table

void l3Init();

// return L2 addr 
// xferMst pass MST
bool getl3Pkt(uint8_t port, L3Pkt* l3Pkt, bool* xferMst, uint8_t * l2Addr);

void l3TxCmplt(L3Pkt* l3pkt);

uint8_t getL3PktFrag(L3Pkt* l3Pkt, uint8_t** ptr, uint8_t idx, uint8_t* txHd, uint8_t* txHdOfst, uint8_t txLen);

bool getL3PktHd(L3Pkt *l3Pkt, uint8_t *hd, uint8_t *ofst);

void l3CmtRx(L3Pkt *l3Pkt, const uint8_t port);

bool l3CmtRxHd(L3Pkt *l3Pkt, const uint8_t port);

uint8_t getL3RxPktFrag(L3Pkt *l3Pkt, uint8_t **ptr, uint8_t idx, uint8_t rxLen);

#endif
