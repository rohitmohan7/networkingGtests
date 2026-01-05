#ifndef L4_NETWORK
#define L4_NETWORK
#include "global.h"

#define MAX_PRIORITY 3

typedef struct __attribute__((packed)) {
    uint8_t msgNo;
    uint16_t len;
} L4Hdr;

typedef struct __attribute__((packed)) {
    L4Hdr hdr;
    uint8_t  head_page;
    uint8_t  tail_page;
    uint8_t  head_off;       /* 0..UNIT-1 */
    uint8_t  tail_used;
} L4Pkt;

typedef struct {
    uint8_t msgNo; // curr Msg No
    uint16_t msgLen; // cur Msg Remaining len cap at uint16_t

    uint8_t head_page;
    uint8_t tail_page;
    uint8_t  head_off;       /* 0..UNIT-1 */
    uint8_t  tail_used;
} prio_stream_t;

typedef struct
{
    uint16_t dst;
    uint16_t gateway;
    /* TCP-ish */
   // uint8_t snd_wnd; // frame windows to send

    /* data in pool (not yet ACKed) */
   // uint16_t queued_bytes;   /* <= POOL_BYTES */
         /* 0..UNIT   */
    prio_stream_t prio[MAX_PRIORITY];
} stream_t;

extern stream_t streams[MAX_POS];

bool getL4Pkt(L4Pkt* l4Pkt, uint8_t portSubnet, uint8_t prio, uint16_t* dstAddr, uint8_t* gatewayL2Addr);

void l4Ack(L4Pkt* l4Pkt, uint8_t prio, uint16_t dstAddr);

void l4Init();
#endif
