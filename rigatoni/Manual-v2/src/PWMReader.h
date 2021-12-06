#ifndef PWMREADER_H
#define PWMREADER_H

#include "mbed.h"

#define MAX_PERIOD_US 20000
#define MAX_WIDTH_US 2000

class PWMReader {

    public:

    PWMReader(PinName pin);
    int64_t getValue();

    private:

    InterruptIn _pin;   // pwm pin
    Timer _timer;
    volatile int64_t _value;
    Timeout _readTimeout;

    /** Start the timer. */
    void startTimer();
    
    /** Reset the timer. */
    void resetTimer();
    
    /** Initialization. */
    void init();
};

#endif
