#ifndef MODULE_ACTUATOR_TORQEEDO_H
#define MODULE_ACTUATOR_TORQEEDO_H

#include <nuclear>

namespace module {
namespace actuator {

    class Torqeedo : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the Torqeedo reactor.
        explicit Torqeedo(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_ACTUATOR_TORQEEDO_H
