#ifndef MODULES_SUPPORT_CONFIGURATION_NETWORKCONFIGURATION_H
#define MODULES_SUPPORT_CONFIGURATION_NETWORKCONFIGURATION_H

#include <nuclear>

namespace module {
namespace support {
namespace configuration {

    class NetworkConfiguration : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the NetworkConfiguration reactor.
        explicit NetworkConfiguration(std::unique_ptr<NUClear::Environment> environment);
    };

}
}
}

#endif  // MODULES_SUPPORT_CONFIGURATION_NETWORKCONFIGURATION_H
