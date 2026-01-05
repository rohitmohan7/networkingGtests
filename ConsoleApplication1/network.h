#ifndef NETWORK
#define NETWORK
#include "global.h"
#include "common.h"

extern uint16_t port_addr[MAX_PORT];

void netInit(UART_Type * UART[MAX_PORT]);

void netTick(uint8_t ms);

static inline uint8_t min(uint8_t a, uint8_t b) { return (a < b) ? a : b; }
#endif