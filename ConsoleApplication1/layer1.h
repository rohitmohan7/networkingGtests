#include "global.h"
#include "common.h"

void l1Init(UART_Type* UARTPtr[MAX_PORT]);

void l1TransferHandleIRQ(UART_Type* UART, uint8_t port);

void l1StartTx(uint8_t port);

void l1RxCmplt(uint8_t port);