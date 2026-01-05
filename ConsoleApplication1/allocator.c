#include "allocator.h"

uint8_t g_next[NUM_PAGES];
uint8_t   g_pool[POOL_BYTES];
uint8_t  g_free_count;
static uint8_t g_free_head;

void pages_init(void)
{
    for (uint8_t i = 0; i < (uint8_t)NUM_PAGES; ++i)
    {
        g_next[i] = (i + 1U < (uint8_t)NUM_PAGES) ? (uint8_t)(i + 1U) : INVALID_PAGE;
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
    return p;
}

static void page_free(uint8_t p)
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