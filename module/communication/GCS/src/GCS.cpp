#include "GCS.h"

#include "extension/Configuration.h"
#include "extension/P2P.h"
#include "message/communication/GamePad.h"
#include "message/propulsion/PropulsionSetpoint.h"
#include "message/propulsion/PropulsionStart.h"
#include "message/propulsion/PropulsionStop.h"
#include "message/communication/GPSTelemetry.h"
#include "message/sensor/GPSRaw.h"
#include "message/status/Mode.h"
#include <functional>
#include <chrono>

namespace module {
namespace communication {

    using extension::Configuration;
    using extension::P2P;
    using message::communication::GamePad;
    using message::propulsion::PropulsionStart;
    using message::propulsion::PropulsionStop;
    using message::communication::GPSTelemetry;
    using message::sensor::GPSRaw;
    using message::status::Mode;

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
            setpoint->port.azimuth = -gamePad.right_analog_stick.x();
            setpoint->starboard.throttle = -gamePad.left_analog_stick.y();
            setpoint->starboard.azimuth = -gamePad.right_analog_stick.x();

           emit(setpoint);
        });

        on<Startup>().then([this]()
        {
            emit(std::make_unique<Mode>(NUClear::clock::now(), Mode::Type::MANUAL));
        });

        on<Trigger<GPSRaw>>().then("GPS Telemetry", [this](const GPSRaw& msg) {

            auto gps = std::make_unique<GPSTelemetry>();
            gps->lat = msg.lla(0);
            gps->lng = msg.lla(1);
            gps->alt = msg.lla(2);
            gps->sats = msg.satellites.size();
            gps->fix = msg.fix_type.value;

            emit<P2P>(gps);
        });
    }
}
}
