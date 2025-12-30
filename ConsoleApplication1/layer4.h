#include "global.h"

#define MAX_PRIORITY 3

typedef struct
{
    /* TCP-ish */
   // uint8_t snd_wnd; // frame windows to send

    /* data in pool (not yet ACKed) */
   // uint16_t queued_bytes;   /* <= POOL_BYTES */
    uint8_t head_page;
    uint8_t tail_page;
    //uint8_t  head_off;       /* 0..UNIT-1 */
    uint8_t  tail_used;      /* 0..UNIT   */

} stream_t;

extern stream_t streams[MAX_POS][MAX_PRIORITY];

