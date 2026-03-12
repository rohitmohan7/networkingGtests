#include <stdint.h>

#define POOL_BYTES   (10240U) // 1K
#define UNIT         (64U)    // 64 bytes
#define NUM_PAGES    (POOL_BYTES / UNIT)

#define INVALID_PAGE ((uint8_t)0xFFU)
extern uint8_t g_next[NUM_PAGES];
extern uint8_t   g_pool[POOL_BYTES];
extern uint8_t  g_free_count;

uint8_t page_alloc(void);
void pages_init(void);
uint8_t ceilPages(uint8_t len);
uint16_t pageOff(uint8_t p);
void page_free(uint8_t p);
