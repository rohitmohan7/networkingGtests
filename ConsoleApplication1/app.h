#include <stdbool.h>
#include <stdint.h>

bool appSend(const uint8_t* data, uint16_t len, uint16_t pos, uint8_t priority, bool retry);

bool appRecv(uint16_t pos, uint8_t priority);

bool appBrdcast(const uint8_t* data, uint16_t len, uint8_t priority);