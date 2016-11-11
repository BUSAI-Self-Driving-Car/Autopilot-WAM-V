#include "GCS.h"

#include "extension/Configuration.h"
#include "extension/P2P.h"
#include "message/communication/GamePad.h"
#include "message/propulsion/PropulsionSetpoint.h"
#include "message/propulsion/PropulsionStart.h"
#include "message/propulsion/PropulsionStop.h"
#include <functional>


namespace module {
namespace communication {

    using extension::Configuration;
    using extension::P2P;
    using message::communication::GamePad;
    using message::propulsion::PropulsionStart;
    using message::propulsion::PropulsionStop;

    GCS::GCS(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
    {
        on<Configuration>("GCS.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file GCS.yaml
        });

        on<P2P<GamePad>>().then("Read", [this](const GamePad& gamePad){

            if (gamePad.A) {
                auto start = std::make_unique<PropulsionStart>();
                emit(start);
                log("Propulsion Start");
            }

            if (gamePad.B) {
                auto stop = std::make_unique<PropulsionStop>();
                emit(stop);
                log("Propulsion Stop");
            }

            auto setpoint = std::make_unique<message::propulsion::PropulsionSetpoint>();
            setpoint->port.throttle = -gamePad.left_analog_stick.y();
            setpoint->port.azimuth = gamePad.right_analog_stick.x();
            setpoint->starboard.throttle = -gamePad.left_analog_stick.y();
            setpoint->starboard.azimuth = gamePad.right_analog_stick.x();

           // log("Game Pad", setpoint->port.throttle, setpoint->port.azimuth, setpoint->starboard.throttle,  setpoint->starboard.azimuth);
            emit(setpoint);
        });
    }
}
}
