#include "GPS.h"

#include "extension/Configuration.h"

namespace module {
namespace sensor {

    using extension::Configuration;

    GPS::GPS(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("GPS.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file GPS.yaml
        });
    }
}
}
