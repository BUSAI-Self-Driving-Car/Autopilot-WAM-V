#include "GCS.h"

#include "extension/Configuration.h"
#include "extension/P2P.h"
#include "message/communication/ControllerCommand.h"
#include "message/propulsion/PropulsionSetpoint.h"
#include <functional>


namespace module {
namespace communication {

    using extension::Configuration;
    using extension::P2P;
    using message::communication::ControllerCommand;

    GCS::GCS(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
    {
        on<Configuration>("GCS.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file GCS.yaml
        });

        on<P2P<ControllerCommand>>().then("Read", [this](const ControllerCommand& controllerCommand){
            auto setpoint = std::make_unique<message::propulsion::PropulsionSetpoint>();
            setpoint->port.throttle = controllerCommand.motor1_thrust;
            setpoint->port.azimuth = controllerCommand.motor1_angle;
            setpoint->starboard.throttle = controllerCommand.motor2_thrust;
            setpoint->starboard.azimuth = controllerCommand.motor2_angle;
            emit(setpoint);
        });
    }
}
}
