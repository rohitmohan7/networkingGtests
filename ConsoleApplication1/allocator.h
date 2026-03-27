#ifndef NETWORK_ALLOCATOR__
#define NETWORK_ALLOCATOR__

#include <stdint.h>
#include <stdbool.h>

#define POOL_BYTES   (10240U) // 1K
#define UNIT         (64U)    // 64 bytes
#define NUM_PAGES    (POOL_BYTES / UNIT)

#define INVALID_PAGE ((uint8_t)0xFFU)
extern uint8_t g_next[NUM_PAGES];
extern uint8_t   g_pool[POOL_BYTES];
extern uint8_t  g_free_count;
#ifdef UNIT_TEST
extern uint8_t g_free_head;
#endif

typedef uint16_t MsgLenType_t;

typedef struct PgPtrHd_st
{
    uint8_t hd;
    uint8_t hdOfst;
} PgPtrHd_t;

typedef struct PgPtrTl_st
{
    uint8_t tl;
    uint8_t tlUsd;
} PgPtrTl_t;

typedef struct PgPtr_st
{
    uint8_t hdPg;
    uint8_t hdOfst; /* 0..UNIT-1 */
    uint8_t tlPg;
    uint8_t tlUsd;
    uint16_t len;
} PgPtr_t;

uint8_t page_alloc(void);
void pages_init(void);
uint8_t ceilPages(uint8_t len);
uint16_t pageOff(uint8_t p);
void page_free(uint8_t p);
uint8_t *getPgPtr(PgPtr_t *pgPtr, uint8_t *len, uint16_t reqLen);

static inline uint16_t min(uint16_t a, uint16_t b) { return (a < b) ? a : b; }
void freePgPtr(PgPtr_t *pgPtr);
void pgPtrInit(PgPtr_t * const pgPtr);
void addUser(PgPtr_t *pgPtr);
uint8_t getPgUsers(uint8_t pg);
void freePgPtrHd(PgPtrHd_t* pgPtrHd, uint16_t len);
void readFromPgs(PgPtr_t* const pgPtr, uint8_t* val, uint16_t size, bool free);
uint16_t  freePgPtrLen(PgPtr_t* pgPtr, uint16_t len);
uint16_t  advancePgPtrLen(PgPtr_t* pgPtr, uint16_t len);
bool allocPgPtr(PgPtr_t* pgPtr, uint16_t len);
void getPgPtrSpan(PgPtr_t* fromPgPtr, PgPtr_t* tpPgPtr, uint16_t start, uint16_t len);
bool allocatorCapacity(PgPtr_t* pgPtr, uint16_t len);
void advancePgPtrHd(PgPtrHd_t* pgPtrHd, uint16_t len);
#endif
