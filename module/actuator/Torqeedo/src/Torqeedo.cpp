#include "Torqeedo.h"

#include "extension/Configuration.h"

namespace module {
namespace actuator {

    using extension::Configuration;

    Torqeedo::Torqeedo(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("Torqeedo.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file Torqeedo.yaml
        });
    }
}
}
