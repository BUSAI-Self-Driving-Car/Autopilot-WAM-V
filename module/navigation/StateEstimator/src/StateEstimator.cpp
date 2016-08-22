#include "StateEstimator.h"

#include "extension/Configuration.h"
#include "message/sensor/IMU.h"
#include "message/sensor/GPS.h"

namespace module {
namespace navigation {

    using extension::Configuration;
    using message::sensor::IMU;
    using message::sensor::GPS;

    StateEstimator::StateEstimator(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , processUpdate(processModel)
        , imuMeasurementUpdate(imuMeasurementModel)
        , gpsMeasurementUpdate(gpsMeasurementModel)
    {

        on<Configuration>("StateEstimator.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file StateEstimator.yaml
        });

        on<Trigger<IMU>>().then("IMU Measurement", [this] (const IMU& meas) {
            //TODO: Update Vehicle State
            log("received IMU:", meas.y.transpose());
        });

        on<Trigger<GPS>>().then("IMU Measurement", [this] (const GPS& meas) {
            //TODO: Update Vehicle State
            log("received GPS:", meas.y.transpose());
        });
    }
}
}
