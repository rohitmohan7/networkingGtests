#include "app.h"
#include "layer4.h"
#include "layer3.h"

bool appSendUdp(const uint8_t* data, const uint16_t len, const uint8_t priority, const uint16_t pos) {
    return l4SendUdp(data, len, priority, pos);
}

#ifdef NETWORK_ISR_RECV
#ifndef UNIT_TEST
void appRecv(uint16_t pos, const uint8_t * const data, uint16_t len) { // TODO
    
    //uint8_t msgType = ; 
}
#endif
#endif
