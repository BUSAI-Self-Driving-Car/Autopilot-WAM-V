#include "PointAndShoot.h"

#include "extension/Configuration.h"
#include "message/status/Mode.h"
#include "message/propulsion/PropulsionSetpoint.h"
#include "utility/Clock.h"
namespace module {
namespace guidance {

    using extension::Configuration;
    using message::status::Mode;
    using message::propulsion::PropulsionSetpoint;

    PointAndShoot::PointAndShoot(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment))
    , stop(true)
    , runTimeSec(0)
    , throttlePercentage(0)
    {

        on<Configuration>("PointAndShoot.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file PointAndShoot.yaml

            stop = true;

            runTimeSec = config["run_time_seconds"];
            throttlePercentage = config["throttle_percentage"];

        });

        on<Trigger<Mode>, Single>().then([this] (const Mode& msg) {

            if (msg.type == Mode::Type::MANUAL) {
                stop = true;
            }
            else if (msg.type == Mode::Type::AUTONOMOUS && stop){
                start_tp = NUClear::clock::now();
                stop = false;
            }
        });

        on<Every<10, std::chrono::milliseconds>>().then([this] {

            if (stop) return;

            auto setpoint = std::make_unique<PropulsionSetpoint>();
            setpoint->port.throttle = throttlePercentage;
            setpoint->port.azimuth = 0;
            setpoint->starboard.throttle = throttlePercentage;
            setpoint->starboard.azimuth = 0;

            auto diff = NUClear::clock::now() - start_tp;
            auto secs = utility::Clock::ToMilli(diff) / 1000.0;

            if (secs > runTimeSec) {
                stop = true;
                setpoint->port.throttle = 0;
                setpoint->starboard.throttle = 0;
            }

            emit(setpoint);

        });

    }
}
}
