#include "NetworkRelay.h"

#include "extension/Configuration.h"
#include "message/sensor/GPSRaw.h"
#include "message/sensor/IMURaw.h"

namespace module {
namespace sensor {

    using extension::Configuration;
    using message::sensor::GPSRaw;
    using message::sensor::IMURaw;

    NetworkRelay::NetworkRelay(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("NetworkRelay.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file NetworkRelay.yaml
        });

        on<Network<IMURaw>>().then([this](const IMURaw& msg) {
            emit(std::make_unique<IMURaw>(msg));
        });

        on<Network<GPSRaw>>().then([this](const GPSRaw& msg) {
            emit(std::make_unique<GPSRaw>(msg));
        });
    }
}
}
