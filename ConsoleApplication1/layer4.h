#ifndef L4_NETWORK
#define L4_NETWORK
#include "global.h"

#define MAX_PRIORITY 3

#define L4_MSG_FLAG_TYPE_ACK 0x1
#define L4_MSG_FLAG_REQ_ACK 0x2
#define L4_MSG_FLAG_STREAM_PENDING 0x4

typedef uint16_t TxOrderType;
typedef uint16_t MsgLenType;

typedef struct __attribute__((packed)) {
    uint8_t msgNo;
    uint8_t msgFlgs;
    MsgLenType msgLen;
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
    TxOrderType txOrder;

    uint8_t head_page;
    uint8_t tail_page;
    uint8_t  head_off;       /* 0..UNIT-1 */
    uint8_t  tail_used;

    uint8_t retryCnt;
    uint8_t retryTmr;
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

void getL4PktFrag(L4Pkt* l4Pkt, uint8_t** ptr, uint8_t* len, uint8_t* txHd, uint8_t* txHdOfst, uint8_t txLen, uint8_t prio);

uint8_t getL4PktHd(L4Pkt* l4Pkt, uint8_t prio, uint8_t* offset);

bool l4StrmPnding(uint8_t pos, uint8_t prio);

bool l4lstMsgFrm(uint16_t pos, uint8_t prio);

void l4Init();
#endif
