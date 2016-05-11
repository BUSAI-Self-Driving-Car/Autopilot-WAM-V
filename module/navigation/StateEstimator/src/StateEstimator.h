#ifndef MODULE_NAVIGATION_STATEESTIMATOR_H
#define MODULE_NAVIGATION_STATEESTIMATOR_H

#include <nuclear>

namespace module {
namespace navigation {

    class StateEstimator : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the StateEstimator reactor.
        explicit StateEstimator(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_NAVIGATION_STATEESTIMATOR_H
