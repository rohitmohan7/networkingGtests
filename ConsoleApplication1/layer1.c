#include "global.h"
#include "layer1.h"
#include "layer2.h"

#define UART_FIFO_SIZE 128

UART_Type* UART[MAX_PORT];

static uint8_t txIndex = 0;

void l2UARTTransferAbortSend(UART_Type* UART) {
	txIndex = 0;
	UART->C2 &= ~((uint8_t)UART_C2_TIE_MASK | (uint8_t)UART_C2_TCIE_MASK);
}

void l1TransferHandleIRQ(UART_Type* UART, uint8_t instance) {
	uint8_t port = instance == 3 ? 0 : 1;
	
	if (((UART_S1_RDRF_MASK & UART->S1) != 0U) && ((UART_C2_RIE_MASK & UART->C2) != 0U)) {
		// abort active transfers
		
		
		// first reset L2 rx timer
		l2TmLstRxRst(port);


	}
}

static void l2UARTWriteNonBlocking(const uint8_t port, const uint8_t* data, size_t length)
{
	assert(data != NULL);

	size_t i;

	/* The Non Blocking write data API assume user have ensured there is enough space in
	peripheral to write. */
	for (i = 0; i < length; i++)
	{
		UART[port]->D = data[i];
	}
}

void l1StartTx(uint8_t port) {
	// request from layer 2 packets
	uint8_t  len;
	do {
		uint8_t* ptr;
		uint8_t  txLen = UART_FIFO_SIZE;
		len = l2GetPkt(ptr, txLen, txIndex); // return remaining len
		l2UARTWriteNonBlocking(port, ptr, len);
		txLen -= len;
	} while (len > 0);
	
	/* Enable transmitter interrupt. */
	UART[port]->C2 |= (UART_C2_TIE_MASK | UART_C2_TE_MASK); // start Tx
}

