#include "IMU.h"

#include "extension/Configuration.h"

namespace module {
namespace sensor {

    using extension::Configuration;

    IMU::IMU(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("IMU.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file IMU.yaml
        });
    }
}
}
