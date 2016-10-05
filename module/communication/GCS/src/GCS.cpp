#include "GCS.h"

#include "extension/Configuration.h"
#include "message/communication/ControllerCommand.h"
#include "message/propulsion/PropulsionSetpoint.h"
#include <functional>

namespace module {
namespace communication {

    using extension::Configuration;
    using message::communication::ControllerCommand;
    using PBControllerCommand = protobuf::message::communication::ControllerCommand;

    GCS::GCS(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , messageParser(uart)
    {
        on<Configuration>("GCS.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file GCS.yaml
            try {
                // Connect to Radio
                std::string radioDevice = config["radio_device"].as<std::string>();
                unsigned int radioBaud = config["radio_baud"].as<unsigned int>();
                uart.close();
                uart.open(radioDevice, radioBaud);
                log<NUClear::INFO>("Open radio on device:", radioDevice," baud:", radioBaud);

                messageParser.registerMessageHandler(MSG_TYPES::CONTROLLER_COMMAND,
                                                     [this] (const std::vector<uint8_t>& buffer) { onControllerCommand(buffer); } );

            }
            catch(std::exception& ex) {
                log<NUClear::ERROR>("Failed to configure: ", ex.what());
            }
        });

        on<IO,Priority::HIGH>(uart.native_handle(), IO::READ).then("Read", [this]{
            messageParser.read();
        });

        on<Shutdown>().then([this]{
            uart.close();
        });
    }

    void GCS::onControllerCommand(const std::vector<uint8_t>& buffer)
    {
        log("Controller Command");
        PBControllerCommand proto;
        proto.ParseFromArray(buffer.data(), buffer.size());
        ControllerCommand controllerCommand = proto;
        log("time_stamp_ms:", controllerCommand.time_stamp_ms);
        log("motor1_thrust:", controllerCommand.motor1_thrust);
        log("motor2_thrust:", controllerCommand.motor2_thrust);
        log("motor1_angle:", controllerCommand.motor1_angle);
        log("motor2_angle:", controllerCommand.motor2_angle);

        auto setpoint = std::make_unique<message::propulsion::PropulsionSetpoint>();
        setpoint->port.throttle = controllerCommand.motor1_thrust;
        setpoint->port.azimuth = controllerCommand.motor1_angle;
        setpoint->starboard.throttle = controllerCommand.motor2_thrust;
        setpoint->starboard.azimuth = controllerCommand.motor2_angle;

        emit(setpoint);
    }
}
}
