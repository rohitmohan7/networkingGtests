#ifndef NETWORK
#define NETWORK

#include "common.h"
#include "config.h"

void netInit(UART_Type * UART[MAX_PORT]);

void netTick();

static inline uint8_t min(uint8_t a, uint8_t b) { return (a < b) ? a : b; }

#if 0
/* TODO move to config.h */
typedef struct
{
    uint8_t subnet[MAX_PORT]; // 0 if unused
} NodeCfg;
#endif

/* TODO move to cfg */
extern NodeCfg_t topology[MAX_POS]; // topology from config
extern PosType_t myPos;

#endif
