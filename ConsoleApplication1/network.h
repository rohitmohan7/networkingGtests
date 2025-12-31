#include "global.h"
#include "common.h"

extern uint16_t port_addr[MAX_PORT];

void netInit(UART_Type * UART[MAX_PORT]);

void netTick(uint8_t ms);