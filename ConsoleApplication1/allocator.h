#ifndef NETWORK_ALLOCATOR__
#define NETWORK_ALLOCATOR__

#include <stdint.h>

#define POOL_BYTES   (10240U) // 1K
#define UNIT         (64U)    // 64 bytes
#define NUM_PAGES    (POOL_BYTES / UNIT)

#define INVALID_PAGE ((uint8_t)0xFFU)
extern uint8_t g_next[NUM_PAGES];
extern uint8_t   g_pool[POOL_BYTES];
extern uint8_t  g_free_count;

typedef struct PgPtr_st
{
    uint8_t hdPg;
    uint8_t tlPg;
    uint8_t hdOfst; /* 0..UNIT-1 */
    uint8_t tlUsd;
} PgPtr_t;

uint8_t page_alloc(void);
void pages_init(void);
uint8_t ceilPages(uint8_t len);
uint16_t pageOff(uint8_t p);
void page_free(uint8_t p);
uint8_t *getPgPtr(PgPtr_t *pgPtr, uint8_t *len, uint8_t reqLen);

static inline uint8_t min(uint8_t a, uint8_t b) { return (a < b) ? a : b; }
void freePgPtr(PgPtr_t *pgPtr);
void pgPtrInit(PgPtr_t * const pgPtr);
void addUser(PgPtr_t *pgPtr);
#endif
