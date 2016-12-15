#include "ErrorForwarder.h"

#include "extension/Configuration.h"

namespace module {
namespace communication {

    using extension::Configuration;

    ErrorForwarder::ErrorForwarder(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("ErrorForwarder.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file ErrorForwarder.yaml
        });
    }
}
}
