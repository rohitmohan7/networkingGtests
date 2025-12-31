#include "global.h"
#include "layer1.h"
#include "layer2.h"
#include <assert.h>

#define UART_FIFO_SIZE 128

UART_Type* UART[MAX_PORT];

static uint8_t txIndex;
static uint8_t rxIndex;

static inline uint8_t min(uint8_t a, uint8_t b) { return (a < b) ? a : b; }

void l1Init(UART_Type* UARTPtr[MAX_PORT]) {
	txIndex = 0;
	rxIndex = 0;
	for (int port = 0; port < MAX_PORT; port++) {
		UART[port] = UARTPtr[port];

		//init registers
	}
}

void l1UARTTransferStopTx(UART_Type* UART) {
	UART->C2 &= ~((uint8_t)UART_C2_TIE_MASK | (uint8_t)UART_C2_TCIE_MASK | (uint8_t)UART_C2_TE_MASK);
}

void l1AbortTx(UART_Type* UART, uint8_t port) {
	l1UARTTransferStopTx(UART);
	l2AbortTx(port);
}

static void l1UARTReadNonBlocking(UART_Type* base, uint8_t* data, size_t length)
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

void l1Rx(UART_Type* UART, uint8_t port) {
	// Tx from layer 2 packets
	uint8_t  len;
	uint8_t  rxLen = UART->RCFIFO;
	do {
		uint8_t* ptr;
		len = l2GetRxPkt(port, ptr, rxLen, rxIndex); // return remaining len
		l1UARTReadNonBlocking(UART, ptr, len);
		rxLen -= len;
		rxIndex += len;
	} while (len > 0 && rxLen > 0);

	if (rxLen > 0) { // not enough mem to write

	}
}

#ifndef UNIT_TEST
static void l1UARTWriteNonBlocking(const uint8_t port, const uint8_t* data, size_t length)
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
#endif

void l1Tx(UART_Type* UART, uint8_t port) {
	uint8_t  txLen = UART_FIFO_SIZE - UART->TCFIFO;
	bool txCmplt;
	do { //  write contigeous buffers into fifo
		uint8_t* ptr;
		uint8_t  len;

		txCmplt = l2GetTxPkt(port, &ptr, &len, txIndex); // return remaining len
		uint8_t txLenMin = min(len, txLen);
		l1UARTWriteNonBlocking(port, ptr, txLenMin);

		txLen -= txLenMin;
		txIndex += txLenMin;
	} while (txLen > 0 && !txCmplt);
	/* Enable transmitter interrupt. */
	if (txCmplt) {
		/* TX register empty interrupt */
		UART->C2 |= (UART_C2_TIE_MASK | UART_C2_TE_MASK); // start Tx
	}
	else {
		/*enable transmission complete interrupt. */
		UART->C2 |= (UART_C2_TCIE_MASK | UART_C2_TE_MASK);
	}
}

static bool l1UARTCmpNonBlocking(UART_Type* base, uint8_t* data, size_t length) {
	assert(data != NULL);

	size_t i;

	/* The Non Blocking read data API assume user have ensured there is enough space in
	peripheral to write. */
	for (i = 0; i < length; i++)
	{
		uint8_t rxData = base->D;
		if (data[i] != rxData) {
			return false;
		}
	}

	return true;
}

void validateTxEcho(UART_Type* UART, uint8_t port, uint8_t count) {
	bool txCmplt;
	do { //  write contigeous buffers into fifo
		uint8_t* ptr;
		uint8_t  len;

		txCmplt = l2GetTxPkt(port, ptr, &len, rxIndex); // return remaining len
		uint8_t rxLenMin = min(len, count);

		bool valid = l1UARTCmpNonBlocking(UART, ptr, rxLenMin);

		count -= len;
		rxIndex += len;

		if (txCmplt && count > 0) { // shouldnt be here since validateTxEcho is invoked when (rxIndex + count) <= txIndex but just incase
			l1AbortTx(UART, port);
			return;
		}

	} while (count > 0);
}

void l1TransferHandleIRQ(UART_Type* UART, uint8_t instance) {
	uint8_t port = instance == 3 ? 0 : 1;

	uint8_t status = UART->S1;
	uint8_t cntrl = UART->C2;
	
	if (((UART_S1_RDRF_MASK & status) != 0U) && ((UART_C2_RIE_MASK & cntrl) != 0U)) {
		uint8_t count = UART->RCFIFO;
		// validate echo
		if ((rxIndex + count) > txIndex) { // tx packet
			if (cntrl & UART_C2_TE_MASK) { // recieved more packets echo will tx still active abort
				l1AbortTx(UART, port);
			}
			else { // proc rx

			}
		} else {
			validateTxEcho(UART, port, count);
		}

		//if (rx_index > tx_index) {
		//	l2UARTTransferAbortSend(UART);
		//}
		
		// first reset L2 rx timer
		l1Rx(UART, port);

		//if (r) {

		//}
	}

	/* Send data register empty and the interrupt is enabled. */
	if (((UART_S1_TDRE_MASK & status) != 0U) && ((UART->C2 & UART_C2_TIE_MASK) != 0U)) {
		l1Tx(UART, port);
	}

	/* Transmission complete and the interrupt is enabled. */
	if ((0U != (UART_S1_TC_MASK & status)) && (0U != (UART->C2 & UART_C2_TCIE_MASK))) {
		l1UARTTransferStopTx(UART);
		//l2TxCmplt(port);
	}
}

void l1StartTx(uint8_t port) {
	UART_Type* txUART = UART[port];
	// Tx from layer 2 packets
	l1Tx(txUART, port);
}

