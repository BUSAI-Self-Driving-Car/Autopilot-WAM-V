#ifndef MODULE_SENSOR_NETWORKRELAY_H
#define MODULE_SENSOR_NETWORKRELAY_H

#include <nuclear>

namespace module {
namespace sensor {

    class NetworkRelay : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the NetworkRelay reactor.
        explicit NetworkRelay(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_SENSOR_NETWORKRELAY_H
