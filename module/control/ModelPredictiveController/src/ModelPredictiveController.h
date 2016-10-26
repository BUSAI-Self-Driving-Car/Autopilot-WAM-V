#ifndef MODULE_CONTROL_MODELPREDICTIVECONTROLLER_H
#define MODULE_CONTROL_MODELPREDICTIVECONTROLLER_H

#include <nuclear>
#include "MPCController.h"

namespace module {
namespace control {

    class ModelPredictiveController : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the ModelPredictiveController reactor.
        explicit ModelPredictiveController(std::unique_ptr<NUClear::Environment> environment);

    private:
        MPCController controller;

    };
}
}

#endif  // MODULE_CONTROL_MODELPREDICTIVECONTROLLER_H
