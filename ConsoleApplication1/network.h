#ifndef NETWORK
#define NETWORK

#include "common.h"
#include "config.h"

void netInit(UART_Type * UART[MAX_PORT]);

void netTick();

/* TODO move to cfg */
extern NodeCfg_t topology[MAX_POS]; // topology from config
extern PosType_t myPos;


#define NETWORK_ISR_RECV
#endif
