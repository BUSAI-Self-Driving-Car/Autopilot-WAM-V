#include "WaypointNav.h"

#include "extension/Configuration.h"

namespace module {
namespace guidance {

    using extension::Configuration;

    WaypointNav::WaypointNav(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("WaypointNav.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file WaypointNav.yaml
        });
    }
}
}
