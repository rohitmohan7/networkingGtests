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
	union {
		PgPtr_t* frwdPktPtr;
		PgPtr_t frwdPkt;
		L4Pkt l4Pkt;
	} pkt;
} L3Pkt;

extern uint16_t l3AddrTblPrio[MAX_POS][MAX_PORT]; // pos addr table ordered by hops/bus load
extern uint16_t l3RouteTable[MAX_SUBNET]; // gateway table
extern uint8_t l3BcastInSubnetForSrcPort[MAX_POS][MAX_PORT];
extern uint8_t l3RouteHops[MAX_SUBNET];

void l3Init(void);

// return L2 addr 
// xferMst pass MST
bool getl3Pkt(uint8_t port, L3Pkt* l3Pkt, bool* xferMst, uint8_t * l2Addr);

void l3TxCmplt(L3Pkt *l3pkt, const uint8_t port);

uint8_t getL3PktFrag(L3Pkt* l3Pkt, uint8_t** ptr, uint8_t idx, uint8_t* txHd, uint8_t* txHdOfst, uint8_t txLen, uint8_t port);

bool getL3PktHd(L3Pkt *l3Pkt, uint8_t *hd, uint8_t *ofst, uint8_t port);

void l3CmtRx(L3Pkt *l3Pkt, const uint8_t port);

bool l3CmtRxHd(L3Pkt *l3Pkt, const uint8_t port);

uint8_t getL3RxPktFrag(uint8_t port, L3Pkt *l3Pkt, uint8_t **ptr, uint8_t rxLen);

uint8_t l3GetTxPktHdrSize(L3Pkt *l3Pkt, uint8_t port);

uint8_t l3GetRxPktHdrSize(L3Pkt *l3Pkt, uint8_t port);

#endif
