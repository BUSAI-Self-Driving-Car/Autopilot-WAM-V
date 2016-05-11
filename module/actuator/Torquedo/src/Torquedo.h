#ifndef MODULE_ACTUATOR_TORQUEDO_H
#define MODULE_ACTUATOR_TORQUEDO_H

#include <nuclear>

namespace module {
namespace actuator {

    class Torquedo : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the Torquedo reactor.
        explicit Torquedo(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_ACTUATOR_TORQUEDO_H
