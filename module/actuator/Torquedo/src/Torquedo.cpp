#include "Torquedo.h"

#include "extension/Configuration.h"

namespace module {
namespace actuator {

    using extension::Configuration;

    Torquedo::Torquedo(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("Torquedo.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file Torquedo.yaml
        });
    }
}
}
