#include "common.h"
#include "pmm.h"

UART_Type* UART0;
UART_Type* UART1;
UART_Type* UART2;
UART_Type* UART3;
UART_Type* UART4;
UART_Type* UART5;

SIM_Type SIMStub;

SIM_Type* SIM = &SIMStub;
GPMM_t g_pmm;