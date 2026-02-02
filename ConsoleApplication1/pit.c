#include "pit.h"
#include <stdlib.h>
/*UT Subset*/

static uint32_t milliSeconds;
static void (*mp_PITTimerCallback[4])(uint8_t);
const int32_t SMC_NUM_PIT_TIMERS = 3;

uint32_t pitGetCurrMS() {
	return milliSeconds;
}

void pitInit() {
    memset(mp_PITTimerCallback, 0, sizeof(mp_PITTimerCallback));
}

bool pitTimespanExceeded(uint32_t startTime, uint32_t endTime, uint32_t duration) {
	return ((endTime - startTime) > duration);
}

void PITCallback(uint8_t channel) // single shot
{
    //Call the callback if set
    if (mp_PITTimerCallback[channel] != NULL)
    {
        mp_PITTimerCallback[channel](channel);
    }
    mp_PITTimerCallback[channel] = NULL;
}

bool PITDisableTimer(int32_t timerNum)
{
    //Check the count
    if (timerNum >= SMC_NUM_PIT_TIMERS)
    {
        return false;
    }

    //Disable timer
 //   PIT->CHANNEL[timerNum].TCTRL = 0;

    return true;
}

bool PITEnableTimerSingleShot(int32_t timerNum, uint32_t cnt, void(*p_callback)(uint8_t))
{
    //Check the count
    if ((timerNum >= SMC_NUM_PIT_TIMERS) || (timerNum < 0))
    {
        return false;
    }
    //Set the callback for this channel
    mp_PITTimerCallback[timerNum] = p_callback;

    // disable the timer from firing again ...

    return true;
}