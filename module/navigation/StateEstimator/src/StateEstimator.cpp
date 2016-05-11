#include "StateEstimator.h"

#include "extension/Configuration.h"

namespace module {
namespace navigation {

    using extension::Configuration;

    StateEstimator::StateEstimator(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("StateEstimator.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file StateEstimator.yaml
        });
    }
}
}
