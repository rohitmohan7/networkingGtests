#include "global.h"

#define MAX_PRIORITY 3



typedef struct {
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

