#ifndef NETWORK
#define NETWORK

#include "common.h"

#define MAX_POS 8

extern uint16_t port_addr[MAX_PORT];

void netInit(UART_Type * UART[MAX_PORT]);

void netTick();

static inline uint8_t min(uint8_t a, uint8_t b) { return (a < b) ? a : b; }

/* TODO move to config.h */
typedef struct
{
    uint8_t subnet[MAX_PORT]; // 0 if unused
} NodeCfg;

extern NodeCfg topology[MAX_POS]; // topology from config

extern uint16_t myPos;

#endif
