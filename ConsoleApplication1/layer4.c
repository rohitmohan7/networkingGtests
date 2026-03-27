//#include "layer4.h"
#include "layer3.h"
#include "allocator.h"
#include "layer2.h"
#include "network.h"
#include "pit.h"
#include "app.h"

#ifndef NETWORK_ISR_RECV
	/* UDP Rx stream */
typedef struct __attribute__((packed)) UdpSock_st {
	PgPtr_t rxPgPtr;
} UdpSock_t;

UdpSock_t udpSck[MAX_POS];

void l4SetUdpTail(PosType_t pos, PgPtr_t* pgPtr) {
	UdpSock_t* udpSckPtr = &udpSck[pos];
	pgPtr->hdPg = pgPtr->tlPg = udpSckPtr->rxPgPtr.tlPg;
	pgPtr->hdOfst = pgPtr->tlUsd = udpSckPtr->rxPgPtr.tlUsd;
}
#endif

void l4Init() {
	
	// init streams
	for (int pos = 0; pos < MAX_POS; pos++) {

#ifndef NETWORK_ISR_RECV
		UdpSock_t* const udpSckPtr = &udpSck[pos];
		pgPtrInit(&udpSckPtr->rxPgPtr);
#endif
	}
	
}

/* before writting here check if there is first check if there is enough pages */
void writeValToPage(PgPtr_t * pgPtr, uint8_t *val, uint8_t len) {
	while (len) {
		uint8_t writeLen;
		uint8_t* ptr = getPgPtr(pgPtr, &writeLen, len);
		writeLen = min(len, writeLen);
		memcpy(ptr, val, writeLen);
		val += writeLen;
		len -= writeLen;
	}
}

#define MAX_UDP_DATAGRAM_SIZE 1000

typedef struct IpTxQueue_t IpTxQueue_t;

bool l4SendUdp(const uint8_t* data, const uint16_t len, const uint8_t priority, const PosType_t pos) {
	/* check first if enough pages are available */
	if (!allocatorCapacity(NULL, len + sizeof(UdpHdr_t))) {
		return false;
	}

	UdpHdr_t udpHdr = {
		.srcPort = myPos,
		.dstPort = pos,
		.length = len
	};

	return l3SendUdp(&udpHdr, data, len, priority);
}

void l4CmtRx(PgPtr_t* const pgPtr, const Protocol_t proto, PosType_t srcPos, MsgLenType_t msgLen) { // recieved a frame

	switch (proto) {
	case IP_PROTO_UDP:
		if (msgLen > sizeof(UdpHdr_t)) {
			msgLen -= sizeof(UdpHdr_t);
			if (msgLen <= MAX_UDP_DATAGRAM_SIZE) {
				/* first read udp header */
				UdpHdr_t udpHdr;
#ifndef NETWORK_ISR_RECV
				/* keep head incase stream is empty */
				PgPtrHd_t pgPtrHd = { .hd = pgPtr->hdPg, .hdOfst = pgPtr->hdOfst };
#endif

				readFromPgs(pgPtr, (uint8_t*)&udpHdr, sizeof(UdpHdr_t),
#ifdef NETWORK_ISR_RECV
				true
#else
                false
#endif
				);

				if (udpHdr.length == msgLen &&
					(udpHdr.dstPort == myPos || udpHdr.dstPort == 0) &&
					(udpHdr.srcPort == srcPos)) {
					/* srcP Port must equal srcPos else something is wrong with addr table or sender */
#ifdef NETWORK_ISR_RECV
					uint8_t udpData[MAX_UDP_DATAGRAM_SIZE];
					readFromPgs(pgPtr, udpData, msgLen, true);
					appRecv(udpHdr.srcPort, udpData, msgLen);
#else
					if (udpSck[srcPos].rxPgPtr.hdPg == INVALID_PAGE) {
						udpSck[srcPos].rxPgPtr.hdPg = pgPtrHd.hd;
						udpSck[srcPos].rxPgPtr.hdOfst = pgPtrHd.hdOfst;
					}
					/* just extend the udp sock tail */
					udpSck[srcPos].rxPgPtr.tlPg = pgPtr->tlPg;
					udpSck[srcPos].rxPgPtr.tlUsd = pgPtr->tlUsd;
#endif
				}
			}
		}
#ifdef NETWORK_ISR_RECV
		freePgPtr(pgPtr);
#endif
		break;
	default:
		break;
	}
}

#ifndef NETWORK_ISR_RECV
uint16_t l4ReadUdp(PosType_t pos, uint8_t * data, uint16_t len) {
	if (pos < MAX_POS) {
		UdpSock_t* udpSock = &udpSck[pos];
		if (udpSock->rxPgPtr.hdPg != INVALID_PAGE) {
			UdpHdr_t udpHdr;
			readFromPgs(&udpSock->rxPgPtr, (uint8_t*)&udpHdr, sizeof(UdpHdr_t), true);
			/* check if we have enough buffer space for full message read */
			if (len >= udpHdr.length) {
				readFromPgs(&udpSock->rxPgPtr, (uint8_t*)data, udpHdr.length, true);
				return udpHdr.length;
			} else {
				/* do partial read and drop rest */
				readFromPgs(&udpSock->rxPgPtr, (uint8_t*)data, len, true);
				freePgPtrLen(&udpSock->rxPgPtr, (udpHdr.length - len));
				return len;
			}
		}
	}
	return 0;
}
#endif