#include "global.h"
#include "layer1.h"
#include "layer2.h"

#define UART_FIFO_SIZE 128

UART_Type* UART[MAX_PORT];

static uint8_t txIndex = 0;
static uint8_t rxIndex = 0;

void l2UARTTransferAbortSend(UART_Type* UART) {
	txIndex = 0;
	UART->C2 &= ~((uint8_t)UART_C2_TIE_MASK | (uint8_t)UART_C2_TCIE_MASK | (uint8_t)UART_C2_TE_MASK);
}

static void l2UARTReadNonBlocking(UART_Type* base, uint8_t* data, size_t length)
{
	assert(data != NULL);

	size_t i;

	/* The Non Blocking read data API assume user have ensured there is enough space in
	peripheral to write. */
	for (i = 0; i < length; i++)
	{
		data[i] = base->D;
	}
}

void l1Rx(UART_Type* UART) {
	// Tx from layer 2 packets
	uint8_t  len;
	uint8_t  rxLen = UART->RCFIFO;
	do {
		uint8_t* ptr;
		len = l2GetRxPkt(port, ptr, rxLen, rxIndex); // return remaining len
		l2UARTReadNonBlocking(UART, ptr, len);
		rxLen -= len;
		rxIndex += len;
	} while (len > 0 && rxLen > 0);
}

void l1TransferHandleIRQ(UART_Type* UART, uint8_t instance) {
	uint8_t port = instance == 3 ? 0 : 1;
	
	if (((UART_S1_RDRF_MASK & UART->S1) != 0U) && ((UART_C2_RIE_MASK & UART->C2) != 0U)) {
		// abort active transfers

		//if (rx_index > tx_index) {
		//	l2UARTTransferAbortSend(UART);
		//}
		
		// first reset L2 rx timer
		l2TmLstRxRst(port);
		l1Rx(UART);

		if (r) {

		}
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
	// Tx from layer 2 packets
	uint8_t  len;
	uint8_t  txLen = UART_FIFO_SIZE;
	do {
		uint8_t* ptr;
		len = l2GetTxPkt(port, ptr, txLen, txIndex); // return remaining len
		l2UARTWriteNonBlocking(port, ptr, len);
		txLen -= len;
		txIndex += len;
	} while (len > 0 && txLen > 0);
	
	/* Enable transmitter interrupt. */
	UART[port]->C2 |= (UART_C2_TIE_MASK | UART_C2_TE_MASK); // start Tx
}

