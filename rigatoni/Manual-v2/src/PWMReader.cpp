#include "PWMReader.h"
 
using namespace std::chrono;


PWMReader::PWMReader(PinName pin) : _pin(pin) {
    init();
}
 
void PWMReader::init() {
    /** configure the rising edge to start the timer */
    _pin.rise(callback(this, &PWMReader::startTimer));
    
    /** configure the falling edge to reset the timer */
    _pin.fall(callback(this, &PWMReader::resetTimer));
    
    _value = -1; // initial value
}
 
void PWMReader::startTimer() {
    _timer.start(); // start the timer
    _readTimeout.attach_us(callback(this, &PWMReader::resetTimer), MAX_PERIOD_US);     
}
 
void PWMReader::resetTimer() {
    // Gets microseconds
    _value = _timer.elapsed_time().count();

    // Clear timeout
    _readTimeout.detach();  
    if(_value > MAX_WIDTH_US)
    {
        _value = 0;
    }

    // reset timer to 0
    _timer.reset(); 
}
 
int64_t PWMReader::getValue() {
    return _value;
}