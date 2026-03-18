#ifndef NETWORK_LAYER1__
#define NETWORK_LAYER1__

#include "common.h"

#define UART_FIFO_SIZE 128

void l1Init(UART_Type* UARTPtr[MAX_PORT]);

void l1TransferHandleIRQ(const UART_Type *const UART, uint8_t port);

void l1StartTx(uint8_t port);

void l1RxCmplt(uint8_t port);
#endif
