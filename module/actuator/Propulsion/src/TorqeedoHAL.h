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
        std::function<void()> watchdog_func;

        static void delay_ms(int x);
        void power_low();
        void power_high();
        void bus_state_receive();
        void bus_state_transmit();
        void packet_transmit (uint8_t *buf, int n);
        void packet_received (TORQEEDO_PACKET_T* packet);

    public:
        TorqeedoHAL(std::string _name,
                    utility::io::uart& _uart,
                    std::function<void()> watchdog);
        void read();
        void start();
        void stop();
        void speed(float);
        void timeout();
    };

}
}

#endif  // MODULE_ACTUATOR_TORQEEDOHAL_H
