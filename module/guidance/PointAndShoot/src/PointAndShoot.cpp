#include "PointAndShoot.h"

#include "extension/Configuration.h"
#include "message/status/Mode.h"
#include "message/propulsion/PropulsionSetpoint.h"
#include "message/control/PositionReference.h"
#include "message/control/TauOveride.h"
#include "utility/Clock.h"
#include "utility/policy/VehicleState.hpp"
#include <opengnc/common/math.hpp>

namespace module {
namespace guidance {

    using extension::Configuration;
    using message::status::Mode;
    using message::propulsion::PropulsionSetpoint;
    using message::control::PositionReference;
    using message::control::TauOveride;
    using message::navigation::StateEstimate;
    using utility::policy::VehicleState;

    PointAndShoot::PointAndShoot(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment))
        , stop(true)
        , runTimeSec(0)
        , throttlePercentage(0)
        , forwardForce(0)
        , heading(0)
        , runMode(RUN_MODE::HEADING)
    {

        on<Configuration>("PointAndShoot.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file PointAndShoot.yaml

            stop = true;

            runTimeSec = config["run_time_seconds"];
            throttlePercentage = config["throttle_percentage"];
            forwardForce = config["forward_force"];
            runMode = static_cast<RUN_MODE>(config["run_mode"].as<int>());
        });

        on<Trigger<Mode>,  With<StateEstimate>, Single>().then([this] (const Mode& msg, const StateEstimate& stateEstimate) {

            if (msg.type == Mode::Type::MANUAL) {
                stop = true;
            }
            else if (msg.type == Mode::Type::AUTONOMOUS && stop){

                auto Rnb = opengnc::common::math::rotationQuaternion(VehicleState::thetanb(stateEstimate.x));
                heading = opengnc::common::math::eulerRotation(Rnb)[2];

                start_tp = NUClear::clock::now();
                stop = false;
            }
        });

        on<Every<10, std::chrono::milliseconds>, With<StateEstimate>>().then([this] (const StateEstimate& stateEstimate) {

            if (stop) return;

            switch (runMode) {
            case RUN_MODE::THROTTLE :
                runthrottleMode(throttlePercentage);
                break;

            case RUN_MODE::HEADING :
                runHeadingMode(forwardForce, stateEstimate);
                break;

            default:
                break;
            }

            auto diff = NUClear::clock::now() - start_tp;
            auto secs = utility::Clock::ToMilli(diff) / 1000.0;

            if (secs > runTimeSec) {
                stop = true;
                runHeadingMode(0, stateEstimate);
                runthrottleMode(0);
            }

        });
    }

    void PointAndShoot::runthrottleMode(double throttle)
    {
        auto setpoint = std::make_unique<PropulsionSetpoint>();
        setpoint->port.throttle = throttle;
        setpoint->port.azimuth = 0;
        setpoint->starboard.throttle = throttle;
        setpoint->starboard.azimuth = 0;
        emit(setpoint);
    }

    void PointAndShoot::runHeadingMode(double force, const StateEstimate &state)
    {
        auto tauOveride = std::make_unique<TauOveride>();
        tauOveride->value << force, 0, 0;
        tauOveride->overide_surge = true;
        tauOveride->overide_sway = false;
        tauOveride->overide_yaw = false;
        emit(tauOveride);

        auto posref = std::make_unique<PositionReference>();
        posref->value << state.x[0], state.x[1], heading;
        emit(posref);
    }

}
}
