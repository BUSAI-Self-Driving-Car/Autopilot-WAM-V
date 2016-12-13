#include "Rudder.h"

#include "message/propulsion/PropulsionStart.h"
#include "message/propulsion/PropulsionSetpoint.h"
#include "extension/Configuration.h"

using namespace module::actuator;

using extension::Configuration;

Rudder::Rudder(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment))
{

    on<Configuration>("Rudder.yaml").then([this] (const Configuration& config)
    {
        // Use configuration here from file Rudder.yaml

        // Setup the port thruster
        const auto port_config = config["port"];
        port.theta_min = port_config["theta_limit"][0].as<float>();
        port.theta_max = port_config["theta_limit"][1].as<float>();
        port.acceleration = port_config["acceleration"].as<uint16_t>();
        port.current_limit = port_config["current_limit"].as<uint16_t>();
        port.high_speed = port_config["high_speed"].as<uint32_t>();
        port.low_speed = port_config["low_speed"].as<uint32_t>();
        port.device = port_config["device"].as<std::string>();
        port.baud = port_config["baud"].as<unsigned int>();

        // Setup the starboard thruster
        const auto starboard_config = config["starboard"];
        starboard.theta_min = starboard_config["theta_limit"][0].as<float>();
        starboard.theta_max = starboard_config["theta_limit"][1].as<float>();
        starboard.acceleration = starboard_config["acceleration"].as<uint16_t>();
        starboard.current_limit = starboard_config["current_limit"].as<uint16_t>();
        starboard.high_speed = starboard_config["high_speed"].as<uint32_t>();
        starboard.low_speed = starboard_config["low_speed"].as<uint32_t>();
        starboard.device = starboard_config["device"].as<std::string>();
        starboard.baud = starboard_config["baud"].as<unsigned int>();
    });

    on<Trigger<message::propulsion::PropulsionSetpoint>>().then([this] (const message::propulsion::PropulsionSetpoint& setpoint)
    {
        port.move(setpoint.port.azimuth);
        starboard.move(setpoint.starboard.azimuth);
    });

    on<Trigger<message::propulsion::PropulsionStart>>().then([this] { start(port); });
    on<Trigger<message::propulsion::PropulsionStart>>().then([this] { start(starboard); });

    on<Watchdog<Stepper<PORT>, 200, std::chrono::milliseconds>>().then([this] { on_watchdog(port); });
    on<Watchdog<Stepper<STARBOARD>, 200, std::chrono::milliseconds>>().then([this] { on_watchdog(starboard); });

    port.negative_home = on<Trigger<Limit<PORT, NEGATIVE>>>().then([this] { negative_limit(port); }).disable();
    starboard.negative_home = on<Trigger<Limit<STARBOARD, NEGATIVE>>>().then([this] { negative_limit(starboard); }).disable();

    port.positive_home = on<Trigger<Limit<PORT, POSITIVE>>, With<StepperPulse<PORT>>>().then([this] (const StepperPulse<PORT>& pulse)
    { positive_limit(port, pulse); }).disable();
    starboard.positive_home = on<Trigger<Limit<STARBOARD, POSITIVE>>, With<StepperPulse<STARBOARD>>>().then([this] (const StepperPulse<STARBOARD>& pulse)
    { positive_limit(starboard, pulse); }).disable();
}

