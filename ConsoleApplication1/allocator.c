#include "allocator.h"

uint8_t g_next[NUM_PAGES];
uint8_t   g_pool[POOL_BYTES];
static uint8_t g_users[NUM_PAGES];
uint8_t  g_free_count;
static uint8_t g_free_head;

void pages_init(void)
{
    for (uint8_t i = 0; i < (uint8_t)NUM_PAGES; ++i)
    {
        g_next[i] = (i + 1U < (uint8_t)NUM_PAGES) ? (uint8_t)(i + 1U) : INVALID_PAGE;
        g_users[i] = 0;
    }
    g_free_head = 0U;
    g_free_count = (uint16_t)NUM_PAGES;
}

uint8_t page_alloc(void)
{
    if (g_free_head == INVALID_PAGE) { return INVALID_PAGE; }
    uint8_t p = g_free_head;
    g_free_head = g_next[p];
    g_next[p] = INVALID_PAGE;
    g_free_count--;
    g_users[p]++;
    return p;
}

void page_free(uint8_t p)
{
    g_next[p] = g_free_head;
    g_free_head = p;
    g_free_count++;
}

uint16_t pageOff(uint8_t p) { return ((uint16_t)p * (uint16_t)UNIT); }

uint8_t ceilPages(uint8_t len)
{
    return ((len + (UNIT - 1U)) / UNIT);
}

uint8_t *getPgPtr(PgPtr_t *pgPtr, uint8_t *len, uint8_t reqLen)
{
    uint8_t *ptr;
    *len = UNIT;

    if (pgPtr->hdPg == INVALID_PAGE)
    {
        pgPtr->hdPg = pgPtr->tlPg = page_alloc();
        const uint16_t base = pageOff(pgPtr->hdPg);
        ptr = &g_pool[base];
        pgPtr->tlUsd = min(reqLen, UNIT);
    }
    else if (pgPtr->tlUsd == UNIT) // need new page
    {
        uint8_t pg = page_alloc();
        g_next[pgPtr->tlPg] = pg;
        pgPtr->tlPg = pg;
        ptr = &g_pool[pageOff(pgPtr->tlPg)];
        pgPtr->tlUsd = min(reqLen, UNIT);
    }
    else
    { // some tail is used
        ptr = &g_pool[(pageOff(pgPtr->tlPg) + pgPtr->tlUsd)];
        *len = (UNIT - pgPtr->tlUsd);
        pgPtr->tlUsd += min(reqLen, *len);
    }
    // update tail offset
    
    return ptr;
}

void freePgPtr(PgPtr_t * pgPtr)
{
    while (pgPtr->hdPg != INVALID_PAGE)
    {
        uint8_t currPage = pgPtr->hdPg;
        pgPtr->hdPg = g_next[currPage];

        g_users[currPage]--;
        if (!g_users[currPage])
        {
            page_free(currPage); // free page if not used by anyone else
        }
    }
    pgPtr->tlPg = pgPtr->hdPg;
}

void addUser(PgPtr_t *pgPtr)
{
    uint8_t currPage = pgPtr->hdPg;
    while (currPage != INVALID_PAGE)
    {
        g_users[currPage]++;
        if (currPage == pgPtr->tlPg)
        { // gate to tail page
            break;
        }
        currPage = g_next[currPage];
    }
}

void pgPtrInit(PgPtr_t * const pgPtr)
{
    pgPtr->hdPg = pgPtr->tlPg = INVALID_PAGE;
    pgPtr->hdOfst = pgPtr->tlUsd = 0;
}
