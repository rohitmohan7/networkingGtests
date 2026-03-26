#include <stdbool.h>
#include <stdint.h>
#include "allocator.h"



bool appSend(const uint8_t* data, uint16_t len, uint16_t pos, uint8_t priority, bool retry);

void appRecv(uint16_t pos, const uint8_t* const data, uint16_t len);

bool appSendUdp(const uint8_t* data, const uint16_t len, const uint8_t priority, const uint16_t pos);