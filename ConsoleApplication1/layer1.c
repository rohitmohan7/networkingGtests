
#include "layer1.h"
#include "layer2.h"
#include "network.h"
#include <assert.h>


UART_Type* UART[MAX_PORT];

static uint16_t txIndex[MAX_PORT];
static uint16_t rxIndex[MAX_PORT];

void l1UARTTransferStopTx(UART_Type* UARTptr) {
	UARTptr->C2 &= ~((uint8_t)UART_C2_TIE_MASK | (uint8_t)UART_C2_TCIE_MASK | (uint8_t)UART_C2_TE_MASK);
}

void l1Init(UART_Type* UARTPtr[MAX_PORT]) {

	for (int port = 0; port < MAX_PORT; port++) {
		UART[port] = UARTPtr[port];
		UART[port]->C2 |= (UART_C2_RE_MASK | UART_C2_RIE_MASK);

		// disable tx
		l1UARTTransferStopTx(UART[port]);

		txIndex[port] = 0;
		rxIndex[port] = 0;
		//init registers
	}
}

static void l1UARTAbortRead(UART_Type* UARTptr)
{
	while ((UARTptr->S1 & UART_S1_RDRF_MASK) != 0U)
	{
		(void)UARTptr->D;
	}

	/* Flush FIFO */
	UARTptr->CFIFO |= UART_CFIFO_RXFLUSH_MASK;
}

#if 0
static inline void l1AbortTx(UART_Type* UARTptr, uint8_t port) {
	txIndex[port] = 0;
	l1UARTTransferStopTx(UARTptr);
	//l2AbortTx(port);
}

static inline void l1AbortRx(UART_Type* UARTptr, uint8_t port) {
	rxIndex[port] = 0;
	l1UARTAbortRead(UARTptr);
	//l2AbortRx(port);
}
#endif

static inline void l1AbortXfer(UART_Type* UARTptr, uint8_t port) {
	txIndex[port] = 0;
	rxIndex[port] = 0;
	l1UARTTransferStopTx(UARTptr);
	l1UARTAbortRead(UARTptr);
	l2AbortXfer(port);
}

void l1TxCmplt(uint8_t port) {
	txIndex[port] = 0;
	rxIndex[port] = 0;
	l2TxCmplt(port);
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
#endif


static inline bool l1Rx(const UART_Type* const UARTptr, uint8_t port) {
	// Tx from layer 2 packets
	uint8_t  rxLen = UARTptr->RCFIFO;
	do {
		uint8_t* ptr;
		uint8_t len = l2GetRxPkt(port, &ptr, rxLen, rxIndex[port]); // return remaining len

		if (len == 0) {
			// abort Rx
			return false;
		}
		uint8_t rxLenMin = min(len, rxLen);
		l1UARTReadNonBlocking(UARTptr, ptr, rxLenMin);
		rxLen -= rxLenMin;
		rxIndex[port] += rxLenMin;
	} while (rxLen > 0);
	return true;
}

void l1Tx(UART_Type* UARTptr, uint8_t port) {
	uint8_t  txLen = UART_FIFO_SIZE - UARTptr->TCFIFO;
	bool txCmplt;
	do { //  write contigeous buffers into fifo
		uint8_t* ptr;
		uint8_t  len;

		txCmplt = l2GetTxPkt(port, &ptr, &len, txIndex[port], txLen, L2_XFER_TX); // return remaining len
		txLen -= len;
		txIndex[port] += len;
		l1UARTWriteNonBlocking(UARTptr, ptr, len);
	} while (txLen > 0 && !txCmplt);
	/* Enable transmitter interrupt. */
	if (txCmplt) {
		/*enable transmission complete interrupt. */
		UARTptr->C2 = ((UARTptr->C2 & ~(uint8_t)UART_C2_TIE_MASK) | UART_C2_TCIE_MASK | UART_C2_TE_MASK);
	}
	else {
		/* TX register empty interrupt */
		UARTptr->C2 |= (UART_C2_TIE_MASK | UART_C2_TE_MASK); // start Tx
	}
}

bool validateTxEcho(UART_Type* UARTptr, uint8_t port, uint8_t count) {
	bool txCmplt;
	do { //  write contigeous buffers into fifo
		uint8_t* ptr = NULL;
		uint8_t  len;

		txCmplt = l2GetTxPkt(port, &ptr, &len, rxIndex[port], count, L2_XFER_RX_ECHO); // return remaining len
		bool valid = l1UARTCmpNonBlocking(UARTptr, ptr, len);

		count = (count > len) ? (count - len) : 0;
		rxIndex[port] += len;

		if (!valid) {
			return false;
		}

		if (txCmplt && !(UARTptr->C2 & UART_C2_TE_MASK)) { // wait for TC complete before 
			l1TxCmplt(port);
		}

	} while (count > 0 && !txCmplt);
	return true;
}

void l1RxCmplt(uint8_t port) {
	rxIndex[port] = 0;
}

void l1TransferHandleIRQ(const UART_Type* const UARTptr, uint8_t port) {
	uint8_t status = UARTptr->S1;
	uint8_t cntrl = UARTptr->C2;
	
	if (((UART_S1_RDRF_MASK & status) != 0U) && ((UART_C2_RIE_MASK & cntrl) != 0U)) {
		uint8_t count = UARTptr->RCFIFO;
		// validate echo
		bool abortXfer = false;
		if ((rxIndex[port] + (uint16_t)count) > txIndex[port]) { // tx packet
			if (cntrl & UART_C2_TE_MASK) { // recieved more packets than echo while tx still active abort
				//l1AbortTx(UARTptr, port);
				//abortRx = true;
				abortXfer = true;
			}
			else { // proc rx
				// validate up to tx index first 
				if (rxIndex[port] < txIndex[port]) {
					abortXfer = !validateTxEcho(UARTptr, port, (txIndex[port] - rxIndex[port]));
				}

				if (!abortXfer) {
					if (l2RxAborted(port)) { // already aborted by l2
						l1UARTAbortRead(UARTptr);
					}
					else {
						abortXfer = !l1Rx(UARTptr, port); // aborted by l2
					}
				}
			}
		} else {
			abortXfer = !validateTxEcho(UARTptr, port, count);
		}

		if (abortXfer) { // error abort
			l1AbortXfer(UARTptr, port);
		}
	}

	/* Send data register empty and the interrupt is enabled. */
	if (((UART_S1_TDRE_MASK & UARTptr->S1) != 0U) && ((UARTptr->C2 & UART_C2_TIE_MASK) != 0U)) {
		l1Tx(UARTptr, port);
	}

	/* Transmission complete and the interrupt is enabled. */
	if ((0U != (UART_S1_TC_MASK & UARTptr->S1)) && (0U != (UARTptr->C2 & UART_C2_TCIE_MASK))) {
		l1UARTTransferStopTx(UARTptr);

		if (rxIndex[port] == txIndex[port]) { // echo complete 
			l1TxCmplt(port);
		}
	}
}

void l1StartTx(uint8_t port) {
	UART_Type* txUART = UART[port];
	//__DMB();                 /* packet visible before enabling ISR NOT NEEDED IF UART ISR PRIO = PIT ISR PRIO */
	txUART->C2 |= (UART_C2_TIE_MASK);
}

