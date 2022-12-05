#include "mbed.h"
#include "rjnet_mbed_udp.h"
#include <VescUart.h>
#include <atomic>
#include <cstdint>

/*
Setup for the Vesc
You cannot use D0, D1 for the serial port. Printf messes with those.
*/
constexpr int BAUD_RATE = 115200;
constexpr PinName TX = D14;
constexpr PinName RX = D15;

constexpr float SI_TO_RPM = 1;
constexpr float MAX_RPM = 4200;
constexpr float GEAR_RATIO = 62.f/22.f;
constexpr float ERPM_TO_RPM = 0.2;

constexpr Kernel::Clock::duration_u32 REFRESH_RATE = 10ms;
constexpr int WATCHDOG_TIMEOUT = 50;

void process_single_message(const SocketAddress & senders_address, const char incoming_udp_message[], unsigned int num_bytes_in_message);

VescUart vesc;
BufferedSerial vesc_serial(TX, RX, BAUD_RATE);
RJNetMbed rjnet_udp(driveIP, &process_single_message);

std::atomic<float> desired_rpm;

template<typename T>
inline int sgn(T n) {
    return (n > 0) - (n < 0);
}

template<typename T>
inline T abs_max_bound(T val, T range) {
    return sgn(val) * min(abs(val), abs(range));
}

void process_single_message(const SocketAddress & senders_address, const char incoming_udp_message[], unsigned int num_bytes_in_message) {
    //Parses a UDP message we just recieved. Places any received data in global variables.
    if(rjnet_udp.are_ip_addrs_equal(nucIP, senders_address)) {
        //Parse angle from message. Doing incoming_message + 2 ignores the first two characters
        if (incoming_udp_message[0] == 'V') {
            float temp;
            sscanf(incoming_udp_message + 2, "%f", &temp);
            //Reply to NUC at once
            char outgoing_message [64];
            sprintf(outgoing_message, "Got V = %f", temp);
            rjnet_udp.send_single_message(outgoing_message, nucIP);
            desired_rpm.store(abs_max_bound(temp * SI_TO_RPM * GEAR_RATIO, MAX_RPM));
        } else if (incoming_udp_message[0] == 'R') {
            NVIC_SystemReset();
        }
    }
}

void watchdog_kick_thread() {
    while (true) {
        Watchdog::get_instance().kick();
        ThisThread::sleep_for(REFRESH_RATE);
    }
}



int main() {
    Watchdog::get_instance().start(WATCHDOG_TIMEOUT);
    Thread watchdog_thread(osPriorityLow);
    watchdog_thread.start(&watchdog_kick_thread);

    DigitalOut led1(LED1);
    DigitalOut led2(LED2);

    if (ResetReason::get() == RESET_REASON_WATCHDOG) {
        led1 = false;
        led2 = true;
    } else {
        led1 = true;
        led2 = false;
    }

    Thread network_start(osPriorityLow);
    network_start.start([&]() { rjnet_udp.start_network_and_listening_threads(); });

    //Set the Vesc's serial port
    vesc.setDebugEnable(false);
    vesc.setSerialPort(&vesc_serial);
    vesc.getVescValues();

    desired_rpm.store(vesc.data.rpm * ERPM_TO_RPM);

    while (true) {
        auto time = Kernel::Clock::now();

        vesc.setRPM(desired_rpm / ERPM_TO_RPM);
        printf("RPM: %f\n", desired_rpm.load());
        ThisThread::sleep_until(time + REFRESH_RATE);
    }
}
