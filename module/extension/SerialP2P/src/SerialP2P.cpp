#include "SerialP2P.h"

#include "extension/Configuration.h"
#include "extension/P2P.h"

namespace module {
namespace extension {

    using ::extension::Configuration;
    using ::extension::P2P;

    SerialP2P::SerialP2P(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("SerialP2P.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file SerialP2P.yaml
        });
    }
}
}
