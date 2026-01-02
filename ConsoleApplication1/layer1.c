#include "global.h"
#include "layer1.h"
#include "layer2.h"
#include <assert.h>

#define UART_FIFO_SIZE 128

UART_Type* UART[MAX_PORT];

static uint8_t txIndex[MAX_PORT];
static uint8_t rxIndex[MAX_PORT];

static inline uint8_t min(uint8_t a, uint8_t b) { return (a < b) ? a : b; }

void l1Init(UART_Type* UARTPtr[MAX_PORT]) {

	for (int port = 0; port < MAX_PORT; port++) {
		UART[port] = UARTPtr[port];
		UART[port]->C2 |= (UART_C2_RE_MASK | UART_C2_RIE_MASK);
		txIndex[port] = 0;
		rxIndex[port] = 0;
		//init registers
	}
}

void l1UARTTransferStopTx(UART_Type* UARTptr) {
	UARTptr->C2 &= ~((uint8_t)UART_C2_TIE_MASK | (uint8_t)UART_C2_TCIE_MASK | (uint8_t)UART_C2_TE_MASK);
}

void l1AbortTx(UART_Type* UARTptr, uint8_t port) {
	txIndex[port] = 0;
	rxIndex[port] = 0;
	l1UARTTransferStopTx(UARTptr);
	l2AbortTx(port);
}

void l1TxCmplt(uint8_t port) {
	txIndex[port] = 0;
	rxIndex[port] = 0;
	l2TxCmplt(port);
}

static void l1UARTReadNonBlocking(UART_Type* UARTptr, uint8_t* data, size_t length)
{
	assert(data != NULL);

	size_t i;

	/* The Non Blocking read data API assume user have ensured there is enough space in
	peripheral to write. */
	for (i = 0; i < length; i++)
	{
		data[i] = UARTptr->D;
	}
}

void l1Rx(UART_Type* UARTptr, uint8_t port) {
	// Tx from layer 2 packets
	uint8_t  len;
	uint8_t  rxLen = UARTptr->RCFIFO;
	do {
		uint8_t* ptr;
		len = l2GetRxPkt(port, ptr, rxLen, rxIndex[port]); // return remaining len
		l1UARTReadNonBlocking(UARTptr, ptr, len);
		rxLen -= len;
		rxIndex[port] += len;
	} while (len > 0 && rxLen > 0);

	if (rxLen > 0) { // not enough mem to write

	}
}

#ifndef UNIT_TEST
static void l1UARTWriteNonBlocking(UART_Type* UARTptr, const uint8_t* data, size_t length)
{
	assert(data != NULL);

	size_t i;

	/* The Non Blocking write data API assume user have ensured there is enough space in
	peripheral to write. */
	for (i = 0; i < length; i++)
	{
		UARTptr->D = data[i];
	}
}
#endif

void l1Tx(UART_Type* UARTptr, uint8_t port) {
	uint8_t  txLen = UART_FIFO_SIZE - UARTptr->TCFIFO;
	bool txCmplt;
	do { //  write contigeous buffers into fifo
		uint8_t* ptr;
		uint8_t  len;

		txCmplt = l2GetTxPkt(port, &ptr, &len, txIndex[port]); // return remaining len
		uint8_t txLenMin = min(len, txLen);
		txLen -= txLenMin;
		txIndex[port] += txLenMin;
		l1UARTWriteNonBlocking(UARTptr, ptr, txLenMin);
		
	} while (txLen > 0 && !txCmplt);
	/* Enable transmitter interrupt. */
	if (txCmplt) {
		/* TX register empty interrupt */
		UARTptr->C2 |= (UART_C2_TIE_MASK | UART_C2_TE_MASK); // start Tx
	}
	else {
		/*enable transmission complete interrupt. */
		UARTptr->C2 |= (UART_C2_TCIE_MASK | UART_C2_TE_MASK);
	}
}

#ifndef UNIT_TEST
static bool l1UARTCmpNonBlocking(UART_Type* UARTptr, uint8_t* data, size_t length) {
	assert(data != NULL);

	size_t i;

	/* The Non Blocking read data API assume user have ensured there is enough space in
	peripheral to write. */
	for (i = 0; i < length; i++)
	{
		uint8_t rxData = UARTptr->D;
		if (data[i] != rxData) {
			return false;
		}
	}

	return true;
}
#endif

void validateTxEcho(UART_Type* UARTptr, uint8_t port, uint8_t count) {
	bool txCmplt;
	do { //  write contigeous buffers into fifo
		uint8_t* ptr;
		uint8_t  len;

		txCmplt = l2GetTxPkt(port, &ptr, &len, rxIndex[port]); // return remaining len
		uint8_t rxLenMin = min(len, count);

		bool valid = l1UARTCmpNonBlocking(UARTptr, ptr, rxLenMin);

		count -= len;
		rxIndex[port] += len;

		if (!valid || (txCmplt && count > 0)) { // shouldnt be here since validateTxEcho is invoked when (rxIndex + count) <= txIndex but just incase
			l1AbortTx(UARTptr, port);
			return;
		}

		if (txCmplt) {
			l1TxCmplt(port);
		}

	} while (count > 0);
}

void l1TransferHandleIRQ(UART_Type* UARTptr, uint8_t instance) {
	uint8_t port = instance == 3 ? 0 : 1;

	uint8_t status = UARTptr->S1;
	uint8_t cntrl = UARTptr->C2;
	
	if (((UART_S1_RDRF_MASK & status) != 0U) && ((UART_C2_RIE_MASK & cntrl) != 0U)) {
		uint8_t count = UARTptr->RCFIFO;
		// validate echo
		if ((rxIndex[port] + count) > txIndex[port]) { // tx packet
			if (cntrl & UART_C2_TE_MASK) { // recieved more packets echo will tx still active abort
				l1AbortTx(UARTptr, port);
			}
			else { // proc rx
				// validate up to tx index first 
				if (rxIndex[port] < txIndex[port]) {
					validateTxEcho(UARTptr, port, txIndex[port]);
				}
				l1Rx(UARTptr, port);
			}
		} else {
			validateTxEcho(UARTptr, port, count);
		}

		//if (rx_index > tx_index) {
		//	l2UARTTransferAbortSend(UART);
		//}
		
		// first reset L2 rx timer
		//

		//if (r) {

		//}
	}

	/* Send data register empty and the interrupt is enabled. */
	if (((UART_S1_TDRE_MASK & status) != 0U) && ((UARTptr->C2 & UART_C2_TIE_MASK) != 0U)) {
		l1Tx(UARTptr, port);
	}

	/* Transmission complete and the interrupt is enabled. */
	if ((0U != (UART_S1_TC_MASK & status)) && (0U != (UARTptr->C2 & UART_C2_TCIE_MASK))) {
		l1UARTTransferStopTx(UARTptr);
		//l2TxCmplt(port);
	}
}

void l1StartTx(uint8_t port) {
	UART_Type* txUART = UART[port];
	// Tx from layer 2 packets
	l1Tx(txUART, port);
}

