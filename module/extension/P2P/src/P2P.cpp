#include "P2P.h"

#include "extension/Configuration.h"

namespace module {
namespace extension {

    using ::extension::Configuration;

    P2P::P2P(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("P2P.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file P2P.yaml
        });
    }
}
}
