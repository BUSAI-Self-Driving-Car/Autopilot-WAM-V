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
            port.torqeedo.reset(new TorqeedoHAL("port",
                                                port.torqeedo_uart,
                                                [this] () { emit(std::make_unique<NUClear::message::ServiceWatchdog<Torqeedo<PORT>>>()); }));
            port.torqeedo_uart.open(port_thruster["torqeedo_device"].as<std::string>(), port_thruster["torqeedo_baud"].as<unsigned int>());

            // Setup the starboard thruster
            const auto starboard_thruster = config["thrusters"]["starboard"];
            starboard.torqeedo.reset(new TorqeedoHAL("starboard",
                                                     starboard.torqeedo_uart,
                                                     [this] () { emit(std::make_unique<NUClear::message::ServiceWatchdog<Torqeedo<STARBOARD>>>()); }));
            starboard.torqeedo_uart.open(starboard_thruster["torqeedo_device"].as<std::string>(), starboard_thruster["torqeedo_baud"].as<unsigned int>());
        });

        on<Watchdog<Torqeedo<PORT>, 200, std::chrono::milliseconds>>().then([this]
        {

            if (!port.reconnecting)
            {
                port.reconnecting = true;
                log<NUClear::WARN>("Port torqeedo timeout. restarting...");
            }

            port.torqeedo->timeout();
        });
        on<Watchdog<Torqeedo<STARBOARD>, 200, std::chrono::milliseconds>>().then([this]
        {
            if (!starboard.reconnecting)
            {
                starboard.reconnecting = true;
                log<NUClear::WARN>("Port torqeedo timeout. restarting...");
            }

            starboard.torqeedo->timeout();
        });

        on<IO,Priority::HIGH>(port.torqeedo_uart.native_handle(), IO::READ).then("port torqeedo read", [this]
        {
            if (port.torqeedo)
            {
                // If we get data we are not restarting anymore
                if (port.reconnecting) {
                    log<NUClear::INFO>("Port Torqeedo reconnected");
                    port.reconnecting = false;
                }

                port.torqeedo->read();
            }
        });

        on<IO,Priority::HIGH>(starboard.torqeedo_uart.native_handle(), IO::READ).then("starboard torqeedo read", [this]
        {
            if (starboard.torqeedo)
            {
                // If we get data we are not restarting anymore
                if (starboard.reconnecting) {
                    log<NUClear::INFO>("Starboard Torqeedo reconnected");
                    starboard.reconnecting = false;
                }

                starboard.torqeedo->read();
            }
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
                case '0':
                {
                    auto setpoint = std::make_unique<message::propulsion::PropulsionSetpoint>();
                    setpoint->port.throttle = 0;
                    setpoint->starboard.throttle = 0;
                    emit(setpoint);
                }
                break;
                case '1':
                {
                    auto setpoint = std::make_unique<message::propulsion::PropulsionSetpoint>();
                    setpoint->port.throttle = 0.1;
                    setpoint->starboard.throttle = 0.1;
                    emit(setpoint);
                }
                break;
                case '2':
                {
                    auto setpoint = std::make_unique<message::propulsion::PropulsionSetpoint>();
                    setpoint->port.throttle = 0.2;
                    setpoint->starboard.throttle = 0.2;
                    emit(setpoint);
                }
                break;
                case '3':
                {
                    auto setpoint = std::make_unique<message::propulsion::PropulsionSetpoint>();
                    setpoint->port.throttle = 0.3;
                    setpoint->starboard.throttle = 0.3;
                    emit(setpoint);
                }
                break;
                case '4':
                {
                    auto setpoint = std::make_unique<message::propulsion::PropulsionSetpoint>();
                    setpoint->port.throttle = 0.4;
                    setpoint->starboard.throttle = 0.4;
                    emit(setpoint);
                }
                break;
                case '5':
                {
                    auto setpoint = std::make_unique<message::propulsion::PropulsionSetpoint>();
                    setpoint->port.throttle = 0.5;
                    setpoint->starboard.throttle = 0.5;
                    emit(setpoint);
                }
                break;
                case '6':
                {
                    auto setpoint = std::make_unique<message::propulsion::PropulsionSetpoint>();
                    setpoint->port.throttle = 0.6;
                    setpoint->starboard.throttle = 0.6;
                    emit(setpoint);
                }
                break;
                case '7':
                {
                    auto setpoint = std::make_unique<message::propulsion::PropulsionSetpoint>();
                    setpoint->port.throttle = 0.7;
                    setpoint->starboard.throttle = 0.7;
                    emit(setpoint);
                }
                break;
                case '8':
                {
                    auto setpoint = std::make_unique<message::propulsion::PropulsionSetpoint>();
                    setpoint->port.throttle = 0.8;
                    setpoint->starboard.throttle = 0.8;
                    emit(setpoint);
                }
                break;
                case '9':
                {
                    auto setpoint = std::make_unique<message::propulsion::PropulsionSetpoint>();
                    setpoint->port.throttle = 0.9;
                    setpoint->starboard.throttle = 0.9;
                    emit(setpoint);
                }
                break;

            }
        });

        on<Trigger<message::propulsion::PropulsionStart> >().then([this] ()
        {
            log("Propulsion Start");

            if (port.torqeedo) { port.torqeedo->speed(0); port.torqeedo->start(); }
        });

        on<Trigger<message::propulsion::PropulsionStart> >().then([this] ()
        {
            if (starboard.torqeedo) { starboard.torqeedo->speed(0); starboard.torqeedo->start(); }
        });

        on<Trigger<message::propulsion::PropulsionStop> >().then([this] ()
        {
            log("STOP");
            if (port.torqeedo) { port.torqeedo->stop(); }
            if (starboard.torqeedo) { starboard.torqeedo->stop(); }
        });

        on<Trigger<message::propulsion::PropulsionSetpoint> >().then([this] (const message::propulsion::PropulsionSetpoint& setpoint)
        {
            if (port.torqeedo) { port.torqeedo->speed(setpoint.port.throttle); }
            if (starboard.torqeedo) { starboard.torqeedo->speed(setpoint.starboard.throttle); }
        });
    }
}
}
