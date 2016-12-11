#include "PathPlanner.h"

#include "extension/Configuration.h"

namespace module {
namespace guidance {

    using extension::Configuration;

    PathPlanner::PathPlanner(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("PathPlanner.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file PathPlanner.yaml
        });
    }
}
}
