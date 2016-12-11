#ifndef MODULE_CONTROL_CONTROLALLOCATION_H
#define MODULE_CONTROL_CONTROLALLOCATION_H

#include <nuclear>
#include "QPControlAllocation.h"

namespace module {
namespace control {

    class ControlAllocation : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the ControlAllocation reactor.
        explicit ControlAllocation(std::unique_ptr<NUClear::Environment> environment);
    private:
        QPControlAllocation qpControlAllocation;
    };

}
}

#endif  // MODULE_CONTROL_CONTROLALLOCATION_H
