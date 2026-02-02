#include <stdint.h>
#include <stdbool.h>

/*UT Subset*/
#define ONE_SECOND 1000 // milliseconds
#define SECONDS(x) (x * ONE_SECOND)

uint32_t pitGetCurrMS();

bool pitTimespanExceeded(uint32_t startTime, uint32_t endTime, uint32_t duration);

void PITCallback(uint8_t channel);

bool PITEnableTimerSingleShot(int32_t timerNum, uint32_t cnt, void(*p_callback)(uint8_t));

bool PITDisableTimer(int32_t timerNum);

void pitInit();