#include "NetworkRelay.h"

#include "extension/Configuration.h"
#include "message/sensor/GPS.h"
#include "message/sensor/IMU.h"

namespace module {
namespace sensor {

    using extension::Configuration;
    using message::sensor::GPS;
    using message::sensor::IMU;

    NetworkRelay::NetworkRelay(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("NetworkRelay.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file NetworkRelay.yaml
        });

        on<Network<IMU>>().then([this](const IMU& msg) {
           emit(std::make_unique<IMU>(msg));
        });

        on<Network<GPS>>().then([this](const GPS& msg) {
           emit(std::make_unique<GPS>(msg));
        });
    }
}
}
