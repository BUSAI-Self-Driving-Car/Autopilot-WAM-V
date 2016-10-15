#ifndef MODULE_ACTUATOR_TORQEEDOHAL_H
#define MODULE_ACTUATOR_TORQEEDOHAL_H

#include <mutex>
#include <utility/io/uart.h>
#include "torqeedo.h"

namespace module {
namespace actuator {

    class TorqeedoHAL
    {
    private:
        static constexpr uint WATCHDOG_PERIOD = 250;
        std::string name;
        utility::io::uart& uart;
        TORQEEDO_DRIVER_T driver;
        std::mutex watchdog_mtx;
        uint watchdog_timer;

        static void delay_ms(int x);
        void service_watchdog();
        void power_low();
        void power_high();
        void bus_state_receive();
        void bus_state_transmit();
        void packet_transmit (uint8_t *buf, int n);
        void packet_received (TORQEEDO_PACKET_T* packet);

    public:
        TorqeedoHAL(std::string _name, utility::io::uart& _uart);
        void watchdog_call(uint ms);
        void read();
        void start();
        void stop();
        void speed(float);
    };

}
}

#endif  // MODULE_ACTUATOR_TORQEEDOHAL_H
