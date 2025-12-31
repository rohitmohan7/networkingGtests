#include "global.h"
#include "common.h"

void l1Init(UART_Type* UARTPtr[MAX_PORT]);

void l1TransferHandleIRQ(UART_Type* UART, uint8_t instance);

void l1StartTx(uint8_t port);