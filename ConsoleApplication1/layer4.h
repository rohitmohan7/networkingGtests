#ifndef L4_NETWORK
#define L4_NETWORK
#include <stdint.h>
#include <stdbool.h>
#include "network.h"
#include "allocator.h"

typedef enum __attribute__((packed)) Protocol_et {
    IP_PROTO_UNKOWN,
    IP_PROTO_UDP,
    IP_PROTO_TCP
} Protocol_t;

typedef struct __attribute__((packed)) UdpHdr_st {
    uint16_t srcPort;
    uint16_t dstPort; // port is pos
    uint16_t length;
    uint16_t checksum; // TODO is this needed since l2 RS485 already check CRC?
} UdpHdr_t;

void l4Init();

void l4CmtRx(PgPtr_t *const pgPtr, const Protocol_t proto, PosType_t srcPos, MsgLenType_t msgLen);

void writeValToPage(PgPtr_t* pgPtr, uint8_t* val, uint8_t len);

bool l4SendUdp(const uint8_t* data, const uint16_t len, const uint8_t priority, const PosType_t pos);

void l4SetUdpTail(PosType_t pos, PgPtr_t* pgPtr);

#ifndef NETWORK_ISR_RECV
uint16_t l4ReadUdp(PosType_t pos, uint8_t* data, uint16_t len);
#endif
#endif
