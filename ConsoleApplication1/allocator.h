#include "global.h"

#define POOL_BYTES   (10240U) // 1K
#define UNIT         (64U)    // 64 bytes
#define NUM_PAGES    (POOL_BYTES / UNIT)

#define INVALID_PAGE ((uint8_t)0xFFFFU)
extern uint8_t g_next[NUM_PAGES];
extern uint8_t   g_tx_pool[POOL_BYTES];
extern uint8_t  g_free_count;