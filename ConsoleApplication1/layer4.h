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

typedef uint16_t TxOrderType;
typedef uint16_t MsgLenType;

typedef struct RxDesc_st {
    uint8_t rxFrmHd;
    uint8_t rxFrmHdOfst;
} RxDesc_t;

typedef struct __attribute__((packed)) {
    uint8_t msgNo;
    uint8_t msgFlgs;
    union {
        MsgLenType msgLen;
        RxDesc_t rxDesc;
    };
} L4Hdr;

#if 0
typedef struct stream_t stream_t;   // forward declaration

typedef struct {
    L4Hdr txMsgHdr;
    TxOrderType txOrder;

    uint8_t head_page;
    uint8_t tail_page;
    uint8_t  head_off;       /* 0..UNIT-1 */
    uint8_t  tail_used;

    uint8_t retryCnt;
    uint8_t retryTmr;
} prio_stream_t;


typedef struct stream_t
{
    prio_stream_t prio[MAX_PRIORITY];
} stream_t;
#endif

typedef struct {
    L4Hdr txMsgHdr;
    L4Hdr rxMsgHdr;

    TxOrderType txOrder;
    uint8_t txFrameCnt;

    uint8_t head_page;
    uint8_t tail_page;
    uint8_t  head_off;       /* 0..UNIT-1 */
    uint8_t  tail_used;
    
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

bool l4StrmEmptyAftFrme(uint16_t pos, uint8_t prio);

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

void l4CmtRx(L4Pkt* l4Pkt, const uint8_t prio, PgPtr_t* pgPtr);

void l4CmtRxHd(L4Pkt *l4Pkt, const uint8_t pos, const uint8_t prio);

void writeValToPage(stream_t *s, uint8_t *val, uint8_t len);

bool l4CmtRxPnding(L4Pkt* l4Pkt);

void l4AbortRx(L4Pkt* l4Pkt, const uint8_t prio);
#endif
