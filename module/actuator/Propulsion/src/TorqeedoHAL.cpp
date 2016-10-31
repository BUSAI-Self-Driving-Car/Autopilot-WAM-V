#include "TorqeedoHAL.h"
#include <thread>
#include <iostream>
#include "message/propulsion/TorqeedoStatus.h"

using namespace module::actuator;

TorqeedoHAL::TorqeedoHAL(std::string _name, utility::io::uart &_uart)
    : name(_name)
    , uart(_uart)
{
    torqeedo_init(&driver, name.c_str(), this);
    driver.delay_ms = delay_ms;
    driver.power_low = [](void* args) { reinterpret_cast<TorqeedoHAL*>(args)->power_low(); };
    driver.power_high = [](void* args) { reinterpret_cast<TorqeedoHAL*>(args)->power_high(); };
    driver.bus_state_receive = [](void* args) { reinterpret_cast<TorqeedoHAL*>(args)->bus_state_receive(); };
    driver.bus_state_transmit = [](void* args) { reinterpret_cast<TorqeedoHAL*>(args)->bus_state_transmit(); };
    driver.packet_transmit = [](void* args, uint8_t* buf, int n) { reinterpret_cast<TorqeedoHAL*>(args)->packet_transmit(buf, n); };
    driver.status_received = [](void* args, const OUTBOARD_STATUS_T* status)
    {
        auto torqeedo_status = std::make_unique<message::propulsion::TorqeedoStatus>();
        torqeedo_status->name = reinterpret_cast<TorqeedoHAL*>(args)->name;
        torqeedo_status->isValid = status->isValid;
        torqeedo_status->motor_voltage = status->motor_voltage;
        torqeedo_status->motor_current = status->motor_current;
        torqeedo_status->motor_power = status->motor_power;
        torqeedo_status->motor_speed = status->motor_speed;
        torqeedo_status->battery_voltage = status->battery_voltage;
        torqeedo_status->battery_current = status->battery_current;
        torqeedo_status->gps_speed = status->gps_speed;

        NUClear::PowerPlant::powerplant->emit(torqeedo_status);
    };
}

void TorqeedoHAL::start()
{
    torqeedo_start(&driver);
}

void TorqeedoHAL::stop()
{
    torqeedo_stop(&driver);
}

void TorqeedoHAL::speed(float x)
{
    torqeedo_set_speed(&driver, x);
}

void TorqeedoHAL::read()
{
    for (int c = uart.get(); c >= 0; c = uart.get())
    {
        torqeedo_recbyte(&driver, c);
    }
}

void TorqeedoHAL::delay_ms (int x)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(x));
}

void TorqeedoHAL::power_low ()
{
    uart.set_dtr(1);
}

void TorqeedoHAL::power_high ()
{
    uart.set_dtr(0);
}

void TorqeedoHAL::bus_state_receive ()
{
    uart.set_rts(1);
}

void TorqeedoHAL::bus_state_transmit ()
{
    uart.set_rts(0);
}

void TorqeedoHAL::packet_transmit (uint8_t *buf, int n)
{
    uart.blocking_write(buf, n);
}
