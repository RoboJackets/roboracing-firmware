#ifndef OdriveMbedSteering_h
#define OdriveMbedSteering_h

#include "OdriveMbedSteeringEnums.h"
#include "BufferedSerial.h"

class ODriveMbed {
    public:
        ODriveMbed(mbed::BufferedSerial& BufferedSerial);

    private:
        std::string readString();
        
        mbed::BufferedSerial& serial_;

};
#endif