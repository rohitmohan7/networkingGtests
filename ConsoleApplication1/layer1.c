#include "global.h"
#include "layer1.h"
#include "layer2.h"
#include "common.h"

void l1TransferHandleIRQ(UART_Type * UART, uint8_t instance) {
	
	uint8_t port = instance == 3 ? 0 : 1;
	
	if (((UART_S1_RDRF_MASK & UART->S1) != 0U) && (UART_C2_RIE_MASK & UART->C2) != 0U)) {
		// first reset L2 rx timer
		l2TmLstRxRst(port);
	}
}