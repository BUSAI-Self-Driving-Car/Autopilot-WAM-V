#include "GCS.h"

#include "extension/Configuration.h"
#include "extension/P2P.h"
#include "message/communication/ControllerCommand.h"

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
            log("Controller Command");
            log("time_stamp_ms:", controllerCommand.time_stamp_ms);
            log("motor1_thrust:", controllerCommand.motor1_thrust);
            log("motor2_thrust:", controllerCommand.motor2_thrust);
            log("motor1_angle:", controllerCommand.motor1_angle);
            log("motor2_angle:", controllerCommand.motor2_angle);
        });
    }
}
}
