#ifndef L4_NETWORK
#define L4_NETWORK
#include <stdint.h>
#include <stdbool.h>
#include "network.h"
#include "allocator.h"

#define MAX_PRIORITY 3

#define L4_MSG_FLAG_TYPE_ACK 0x1
#define L4_MSG_FLAG_REQ_ACK 0x2
#define L4_MSG_FLAG_STREAM_PENDING 0x4
#define L4_MSG_FLAG_PENDING_ACK 0x8
#define L4_MSG_FLAG_RXHD_CMT 0x10

#define MAX_L4_RETRY 3
#define L4_RETRY_TIMEOUT 200

typedef enum Protocol_et {
    IP_PROTO_UNKOWN,
    IP_PROTO_UDP,
    IP_PROTO_TCP
} Protocol_t;

typedef uint16_t TxOrderType;


typedef struct __attribute__((packed)) {
    uint8_t msgNo;
    uint8_t msgFlgs;
    union {
        uint16_t msgLen;
        PgPtrHd_t rxDesc;
    };
} L4Hdr;

typedef struct {
    L4Hdr txMsgHdr;
    L4Hdr rxMsgHdr;

    TxOrderType txOrder;
    uint8_t txFrameCnt;

    PgPtr_t txPgPtr;

    // to do use array ?
    PgPtr_t rxPgPtr;

    uint8_t retryCnt;
    uint32_t retryTmr;
} stream_t;

typedef struct __attribute__((packed)) {
    L4Hdr hdr;
    uint16_t pos;
} L4Pkt;

extern TxOrderType txOrder;

//extern stream_t streams[MAX_POS];
extern stream_t streams[MAX_POS][MAX_PRIORITY];

//bool getL4Pkt(L4Pkt* l4Pkt, uint8_t portSubnet, uint8_t prio, uint16_t* dstAddr);
bool getL4Pkt(L4Pkt* l4Pkt, uint16_t pos, uint8_t prio, uint8_t pktPrio);

bool l4StrmEmpty(uint16_t pos, uint8_t prio);

bool l4StrmEmptyAftUnicstFrme(uint16_t pos, uint8_t prio);

void setL4Hdr(L4Pkt* l4Pkt, uint8_t prio);

bool l4Ack(L4Pkt* l4Pkt, uint8_t prio, uint16_t dstAddr);

void l4TxCmplt(L4Pkt* l4Pkt, uint8_t prio);

uint8_t getL4PktFrag(L4Pkt* l4Pkt, uint8_t** ptr, uint8_t idx, uint8_t* txHd, uint8_t* txHdOfst, uint8_t txLen, uint8_t prio);

uint8_t getL4RxPktFrag(L4Pkt *l4Pkt, uint8_t **ptr, uint8_t rxLen, uint8_t prio);

bool getL4PktHd(L4Pkt *l4Pkt, uint8_t prio, uint8_t *hd, uint8_t *offset);

bool l4StrmPnding(uint8_t pos, uint8_t prio);

bool l4lstMsgFrm(uint16_t pos, uint8_t prio);

void l4Init();

void l4Tick(uint8_t ms);

void l4CmtRx(PgPtr_t *const pgPtr, const Protocol_t proto, MsgLenType_t msgLen);

void l4CmtRxHd(L4Pkt *l4Pkt, const uint8_t pos, const uint8_t prio);

void writeValToPage(PgPtr_t* pgPtr, uint8_t* val, uint8_t len);

bool l4CmtRxPnding(L4Pkt* l4Pkt);

void l4RxGetLastFrame(const uint8_t prio, L4Pkt *l4Pkt, PgPtr_t *frame);

void setL4HdrBrdcst(L4Hdr* l4PktHdr, L4Hdr* txHdr);

bool l4lstBrdcstMsgFrm(L4Hdr* l4Pkt);

void l4SetBrdcastStrmPnding(L4Hdr* hdr, const bool pending);

bool l4TxBrdcstStrmPnding(L4Hdr* l4Pkt);

bool l4TxBrdcastStrmEmpty(PgPtr_t* pgPtr);

bool l4StrmEmptyAftBrdcstFrme(const PgPtr_t* const pgPtr, const MsgLenType_t msgLen);

TxOrderType l4GetStrmTxOrder(const uint8_t pos, const uint8_t prio);
#endif
