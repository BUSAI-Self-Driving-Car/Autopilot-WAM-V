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
        port.uart.open(port_config["device"].as<std::string>(), port_config["baud"].as<unsigned int>());

        // Setup the starboard thruster
        const auto starboard_config = config["starboard"];
        starboard.theta_min = starboard_config["theta_limit"][0].as<float>();
        starboard.theta_max = starboard_config["theta_limit"][1].as<float>();
        starboard.acceleration = starboard_config["acceleration"].as<uint16_t>();
        starboard.current_limit = starboard_config["current_limit"].as<uint16_t>();
        starboard.high_speed = starboard_config["high_speed"].as<uint32_t>();
        starboard.low_speed = starboard_config["low_speed"].as<uint32_t>();
        starboard.uart.open(starboard_config["device"].as<std::string>(), starboard_config["baud"].as<unsigned int>());

        port.uart_handle.unbind();
        port.uart_handle = on<IO,Priority::HIGH>(port.uart.native_handle(), IO::READ).then("port stepper read", [this] { read_uart(port); });

        port.uart_handle.unbind();
        starboard.uart_handle = on<IO,Priority::HIGH>(starboard.uart.native_handle(), IO::READ).then("starboard stepper read",  [this] { read_uart(starboard); });
    });

    on<Trigger<message::propulsion::PropulsionStart>>().then([this]
    {
        // Configure the stepper driver
        port.queue_command("ABS", true);
        port.queue_command("ACC=" + std::to_string(port.acceleration), true);
        port.queue_command("DRVIC="+std::to_string(port.current_limit), true);
        port.queue_command("DVRMS=2", true);
        port.queue_command("HSPD="+std::to_string(port.high_speed), true);
        port.queue_command("LSPD="+std::to_string(port.low_speed), true);
        port.queue_command("RW", true);

        // Go to negative limit
        port.queue_command("L-", true);

        port.negative_home.enable();
        port.positive_home.enable();
    });

    on<Trigger<message::propulsion::PropulsionSetpoint>>().then([this] (const message::propulsion::PropulsionSetpoint& setpoint)
    {

    });

    on<Watchdog<Stepper<PORT>, 200, std::chrono::milliseconds>>().then([this]
    {
        log<NUClear::WARN>("Stepper", int(port.side), "transmit timeout");
        write_command<Stepper<PORT>>(port);
    });

    port.negative_home = on<Trigger<Limit<PORT, NEGATIVE>>>().then([this]
    {
        log<NUClear::INFO>("Port negative limit reached");
        port.queue_command("J+", true);

        port.negative_home.disable();
    }).disable();

    port.positive_home = on<Trigger<Limit<PORT, POSITIVE>>, With<StepperPulse<PORT>>>().then([this] (const StepperPulse<PORT>& pulse)
    {
        port.pulse_min = -pulse.count/2;
        port.pulse_max = pulse.count/2;
        port.conversion = pulse.count / (port.theta_max - port.theta_min);

        port.target = 0;
        port.queue_command("PX="+std::to_string(pulse.count/2), true);
        port.queue_command("X"+std::to_string(port.target), true);

        port.positive_home.disable();
    }).disable();
}

