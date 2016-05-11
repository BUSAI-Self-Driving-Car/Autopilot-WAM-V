#include "ModelPredictiveController.h"

#include "extension/Configuration.h"

namespace module {
namespace control {

    using extension::Configuration;

    ModelPredictiveController::ModelPredictiveController(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("ModelPredictiveController.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file ModelPredictiveController.yaml
        });
    }
}
}
