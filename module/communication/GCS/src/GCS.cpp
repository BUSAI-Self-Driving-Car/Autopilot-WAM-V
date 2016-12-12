#include "GCS.h"

#include "extension/Configuration.h"
#include "extension/P2P.h"
#include "message/communication/GamePad.h"
#include "message/control/Tau.h"
#include "message/propulsion/PropulsionSetpoint.h"
#include "message/propulsion/PropulsionStart.h"
#include "message/propulsion/PropulsionStop.h"
#include "message/communication/GPSTelemetry.h"
#include "message/sensor/GPSRaw.h"
#include "message/status/Mode.h"
#include "message/navigation/StateEstimate.h"
#include "utility/policy/VehicleState.hpp"
#include <opengnc/common/math.hpp>
#include <functional>
#include <chrono>

namespace module {
namespace communication {

    using extension::Configuration;
    using extension::P2P;
    using message::communication::GamePad;
    using message::control::Tau;
    using message::propulsion::PropulsionSetpoint;
    using message::propulsion::PropulsionStart;
    using message::propulsion::PropulsionStop;
    using message::communication::GPSTelemetry;
    using message::sensor::GPSRaw;
    using message::status::Mode;
    using message::communication::Status;
    using message::navigation::StateEstimate;
    using StatePolicy = utility::policy::VehicleState;

    GCS::GCS(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
    {
        on<Configuration>("GCS.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file GCS.yaml
            manual_mode_type = config["manual_mode_type"];

            auto setpoint = std::make_unique<PropulsionSetpoint>();
            setpoint->port.throttle = 0;
            setpoint->port.azimuth = 0;
            setpoint->starboard.throttle = 0;
            setpoint->starboard.azimuth = 0;

            emit(setpoint);
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

            if (manual_mode_type == 1) {
                auto setpoint = std::make_unique<PropulsionSetpoint>();
                setpoint->port.throttle = -gamePad.left_analog_stick.y();
                setpoint->port.azimuth = -gamePad.right_analog_stick.x();
                setpoint->starboard.throttle = -gamePad.left_analog_stick.y();
                setpoint->starboard.azimuth = -gamePad.right_analog_stick.x();

                emit(setpoint);

            }
            else if (manual_mode_type == 2) {

                Eigen::Vector3d input;
                input << -1200 * gamePad.left_analog_stick.y(),
                        -600 * gamePad.left_analog_stick.x(),
                        -1200 * gamePad.right_analog_stick.x();

                auto tau = std::make_unique<Tau>();
                tau->value  = input;
                emit(tau);
            }
        });

        on<Startup>().then([this]()
        {
            emit<Scope::LOCAL, Scope::NETWORK>(std::make_unique<Mode>(NUClear::clock::now(), Mode::Type::MANUAL));
        });

        on<Trigger<GPSRaw>, Sync<GCS>>().then("GPS Telemetry", [this](const GPSRaw& msg) {
            lastStatus.lat = msg.lla(0);
            lastStatus.lng = msg.lla(1);
            lastStatus.alt = msg.lla(2);
            lastStatus.sats = msg.satellites.size();
            lastStatus.fix = msg.fix_type.value;
        });

        on<Trigger<StateEstimate>, Sync<GCS>>().then("State Estimate Telemetry", [this](const StateEstimate& msg) {
            lastStatus.north = StatePolicy::rBNn(msg.x)[0];
            lastStatus.east = StatePolicy::rBNn(msg.x)[1];
            lastStatus.heading = opengnc::common::math::eulerRotation(StatePolicy::Rnb(msg.x))[2];
            lastStatus.surge_vel = StatePolicy::vBNb(msg.x)[0];
            lastStatus.sway_vel = StatePolicy::vBNb(msg.x)[1];
            lastStatus.yaw_rate = StatePolicy::omegaBNb(msg.x)[2];
        });

        on<Every<1, std::chrono::seconds>, Sync<GCS>>().then([this] {
            emit<P2P>(std::make_unique<Status>(lastStatus));
        });
    }
}
}
