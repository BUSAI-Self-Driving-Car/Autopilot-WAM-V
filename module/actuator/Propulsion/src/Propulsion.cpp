#include "Propulsion.h"

#include "extension/Configuration.h"
#include "message/propulsion/PropulsionSetpoint.h"
#include "message/propulsion/PropulsionStart.h"
#include "message/propulsion/PropulsionStop.h"

namespace module {
namespace actuator {

    using extension::Configuration;

    Propulsion::Propulsion(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("Propulsion.yaml").then([this] (const Configuration& config)
        {
            // Setup the port thruster
            const auto port_thruster = config["thrusters"]["port"];
            port.torqeedo.reset(new TorqeedoHAL("port", port.torqeedo_uart));
            port.torqeedo_uart.open(port_thruster["torqeedo_device"].as<std::string>(), port_thruster["torqeedo_baud"].as<unsigned int>());
            port.stepper.reset(new Stepper(port.stepper_uart));
            port.stepper_uart.open(port_thruster["stepper_device"].as<std::string>(), port_thruster["stepper_baud"].as<unsigned int>());
            port.stepper->begin();

            // Setup the starboard thruster
            const auto starboard_thruster = config["thrusters"]["starboard"];
            starboard.torqeedo.reset(new TorqeedoHAL("starboard", starboard.torqeedo_uart));
            starboard.torqeedo_uart.open(starboard_thruster["torqeedo_device"].as<std::string>(), starboard_thruster["torqeedo_baud"].as<unsigned int>());
            starboard.stepper.reset(new Stepper(starboard.stepper_uart));
            starboard.stepper_uart.open(starboard_thruster["stepper_device"].as<std::string>(), starboard_thruster["stepper_baud"].as<unsigned int>());
            starboard.stepper->begin();
        });

        on<IO,Priority::HIGH>(port.torqeedo_uart.native_handle(), IO::READ).then("port torqeedo read", [this]
        {
            if (port.torqeedo) { port.torqeedo->read(); }
        });

        on<IO,Priority::HIGH>(starboard.torqeedo_uart.native_handle(), IO::READ).then("starboard torqeedo read", [this]
        {
            if (starboard.torqeedo) { starboard.torqeedo->read(); }
        });

        on<IO,Priority::HIGH>(port.stepper_uart.native_handle(), IO::READ).then("port stepper read", [this]
        {
            if (port.stepper) { port.stepper->read(); }
        });

        on<IO,Priority::HIGH>(starboard.stepper_uart.native_handle(), IO::READ).then("starboard stepper read", [this]
        {
            if (starboard.stepper) { starboard.stepper->read(); }
        });

        on<Every<10, std::chrono::milliseconds>>().then([this] ()
        {
           if (port.torqeedo) { port.torqeedo->watchdog_call(10); }
           if (starboard.torqeedo) { starboard.torqeedo->watchdog_call(10); }
        });

        on<IO>(STDIN_FILENO, IO::READ).then([this]()
        {
            char c;
            std::cin >> c;

            switch (c)
            {
                case 's':
                {
                    auto start = std::make_unique<message::propulsion::PropulsionStart>();
                    emit(start);
                }
                break;
                case ' ':
                {
                    auto stop = std::make_unique<message::propulsion::PropulsionStop>();
                    emit(stop);
                }
                break;
            }
        });

        on<Trigger<message::propulsion::PropulsionStart> >().then([this] ()
        {
            log("Propulsion Start");

            if (port.torqeedo) { port.torqeedo->start(); }
            if (port.stepper)
            {
                port.stepper->motorEnable(true);
                if (!port.stepper->isHomed()) { port.stepper->motorHome(); }
            }

            if (starboard.torqeedo) { starboard.torqeedo->start(); }
            if (starboard.stepper)
            {
                starboard.stepper->motorEnable(true);
                if (!starboard.stepper->isHomed()) { starboard.stepper->motorHome(); }
            }
        });

        on<Trigger<message::propulsion::PropulsionStop> >().then([this] ()
        {
            log("STOP");
            if (port.torqeedo) { port.torqeedo->stop(); }
            if (port.stepper) { port.stepper->motorStop(true); }

            if (starboard.torqeedo) { starboard.torqeedo->stop(); }
            if (starboard.stepper) { starboard.stepper->motorStop(true); }

            // TODO: Make more elegant
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            if (port.stepper) { port.stepper->motorEnable(false); }
            if (starboard.stepper) { starboard.stepper->motorEnable(false); }
        });

        on<Trigger<message::propulsion::PropulsionSetpoint> >().then([this] (const message::propulsion::PropulsionSetpoint& setpoint)
        {
            return;
            if (port.torqeedo) { port.torqeedo->speed(setpoint.port.throttle); }
            if (port.stepper) { port.stepper->azimuth(setpoint.port.azimuth); }

            if (starboard.torqeedo) { starboard.torqeedo->speed(setpoint.starboard.throttle); }
            if (starboard.stepper) { starboard.stepper->azimuth(setpoint.starboard.azimuth); }
        });

        on<Every<50, std::chrono::milliseconds>>().then([this] ()
        {
            if (port.stepper) { port.stepper->run(); }
            if (starboard.stepper) { starboard.stepper->run(); }
        });

        on<Every<2000, std::chrono::milliseconds>>().then([this] ()
        {
            if (port.stepper) { port.stepper->motorStatus(); }
            if (starboard.stepper) { starboard.stepper->motorStatus(); }
        });
    }
}
}
