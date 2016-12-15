#include "GCS.h"

#include "extension/Configuration.h"
#include "extension/P2P.h"
#include "message/communication/GamePad.h"
#include "message/control/Tau.h"
#include "message/propulsion/PropulsionSetpoint.h"
#include "message/propulsion/PropulsionStart.h"
#include "message/propulsion/PropulsionStop.h"
#include "message/communication/GPSTelemetry.h"
#include "message/communication/GCSMessage.h"
#include "message/sensor/GPSRaw.h"
#include "message/sensor/IMURaw.h"
#include "message/status/Mode.h"
#include "message/navigation/StateEstimate.h"
#include "utility/policy/VehicleState.hpp"
#include <opengnc/common/math.hpp>
#include <functional>
#include <chrono>
#include "utility/Clock.h"

namespace module {
namespace communication {

    using NUClear::message::LogMessage;
    using extension::Configuration;
    using extension::P2P;
    using message::communication::GamePad;
    using message::control::Tau;
    using message::propulsion::PropulsionSetpoint;
    using message::propulsion::PropulsionStart;
    using message::propulsion::PropulsionStop;
    using message::communication::GPSTelemetry;
    using message::sensor::GPSRaw;
    using message::sensor::IMURaw;
    using message::status::Mode;
    using message::communication::Status;
    using message::communication::GCSMessage;
    using message::navigation::StateEstimate;
    using StatePolicy = utility::policy::VehicleState;
    using utility::Clock;

    GCS::GCS(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , mode(Mode::Type::MANUAL)
    {
        lastStatus.num = 0;

        on<Configuration>("GCS.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file GCS.yaml
            manual_mode_type = config["manual_mode_type"];

            auto setpoint = std::make_unique<PropulsionSetpoint>();
            setpoint->port.throttle = 0;
            setpoint->port.azimuth = 0;
            setpoint->starboard.throttle = 0;
            setpoint->starboard.azimuth = 0;

            mode = Mode::Type::MANUAL;

            emit(setpoint);
        });

        on<P2P<GamePad>>().then("Read", [this](const GamePad& gamePad) {

            if (gamePad.LB && gamePad.down){
                mode = Mode::Type::AUTONOMOUS;
                emitMode();
            }
            if (gamePad.RB) {
                mode = Mode::Type::MANUAL;
                emitMode();
            }

            if (mode == Mode::Type::MANUAL) {

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
                               600 * gamePad.left_analog_stick.x(),
                              1200 * gamePad.right_analog_stick.x();

                    auto tau = std::make_unique<Tau>();
                    tau->value  = input;
                    emit(tau);
                }
            }
        });

        on<Startup>().then([this]()
        {
            emit<Scope::LOCAL, Scope::NETWORK>(std::make_unique<Mode>(NUClear::clock::now(), Mode::Type::MANUAL));
        });

        on<Trigger<IMURaw>, Sync<GCS>>().then([this](const IMURaw& msg) {
            lastStatus.imu_feq += 1;
        });

        on<Trigger<GPSRaw>, Sync<GCS>>().then("GPS Telemetry", [this](const GPSRaw& msg) {
            lastStatus.lat = msg.lla(0);
            lastStatus.lng = msg.lla(1);
            lastStatus.alt = msg.lla(2);
            lastStatus.sats = msg.satellites.size();
            lastStatus.fix = msg.fix_type.value;
            lastStatus.gps_feq += 1;
        });

        on<Trigger<StateEstimate>, Sync<GCS>>().then("State Estimate Telemetry", [this](const StateEstimate& msg) {
            lastStatus.north = StatePolicy::rBNn(msg.x)[0];
            lastStatus.east = StatePolicy::rBNn(msg.x)[1];
            lastStatus.heading = opengnc::common::math::eulerRotation(StatePolicy::Rnb(msg.x))[2] * 180.0/M_PI;
            lastStatus.surge_vel = StatePolicy::vBNb(msg.x)[0];
            lastStatus.sway_vel = StatePolicy::vBNb(msg.x)[1];
            lastStatus.yaw_rate = StatePolicy::omegaBNb(msg.x)[2];
            lastStatus.state_est_feq += 1;
        });

        on<Trigger<PublishMessage>, Sync<GCS>>().then([this] (const PublishMessage& message) {
            emit<P2P>(std::make_unique<GCSMessage>(message.str));
        });

        on<Every<500, std::chrono::milliseconds>>().then([this] () {
            std::lock_guard<std::mutex> lg(message_mutex);
            if (message_queue.size() > 0)
            {
                const auto msg = message_queue.front();
                message_queue.pop();
                emit(std::make_unique<PublishMessage>(msg));
                if (dropped_messages > 0)
                {
                    message_queue.push("Dropped " + std::to_string(dropped_messages) + " messages.");
                    dropped_messages = 0;
                }
            }
        });

        on<Network<GCSMessage>>().then([this] (const GCSMessage& message) { emit(std::make_unique<GCSMessage>(message)); });
        on<Trigger<GCSMessage>>().then([this] (const GCSMessage& message)
        {
            if (message_queue.size() < MAXIMUM_QUEUE_SIZE)
            {
                std::lock_guard<std::mutex> lg(message_mutex);
                message_queue.push(message.str);
            }
            else
            {
                dropped_messages++;
            }
        });

        on<Trigger<LogMessage>>().then([this] (const LogMessage& message) {

            // Where this message came from
            std::string source = "";

            // If we know where this log message came from, we display that
            if (message.task) {
                // Get our reactor name
                std::string reactor = message.task->identifier[1];

                // Strip to the last semicolon if we have one
                size_t lastC = reactor.find_last_of(':');
                reactor = lastC == std::string::npos ? reactor : reactor.substr(lastC + 1);

                // This is our source
                source = reactor + " " + (message.task->identifier[0].empty() ? "" : "- " + message.task->identifier[0] + " ");
            }

            std::string str(source);

            // Output the level
            switch(message.level)
            {
                case NUClear::ERROR:
                    str += "ERROR: " + message.message;
                    emit(std::make_unique<GCSMessage>(str));
                    break;
                case NUClear::FATAL:
                    str += "FATAL: " + message.message;
                    emit(std::make_unique<GCSMessage>(str));
                    break;
            case NUClear::TRACE:
            case NUClear::DEBUG:
            case NUClear::INFO:
            case NUClear::WARN:
                break;
            }
        });

        on<Every<1, std::chrono::seconds>, Sync<GCS>>().then([this] {
            lastStatus.num += 1;
            emit<P2P>(std::make_unique<Status>(lastStatus));
            lastStatus.gps_feq = 0;
            lastStatus.imu_feq = 0;
            lastStatus.state_est_feq = 0;
        });
    }

    void GCS::emitMode()
    {
        lastStatus.mode = mode == Mode::Type::AUTONOMOUS ? 2 : 1;
        auto msg = std::make_unique<Mode>();
        msg->type = mode;
        emit<Scope::NETWORK, Scope::LOCAL>(msg);
    }
}
}
