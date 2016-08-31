#include "StateEstimator.h"

#include "utility/convert/yaml-eigen.h"
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

            using namespace Eigen;
            // IMU Parameters

//            imuMeasurementModel.set_mag_scale(config["mag_scale"].as<Vector3d>());
//            imuMeasurementModel.set_mag_vector(config["mag_vector"].as<Vector3d>());
//            imuMeasurementModel.set_Rib(config["mag_vector"].as<Matrix3d>());
//            imuMeasurementModel.set_rIBb(config["rIBb"].as<Vector3d>());
//            imuMeasurementModel.set_gn(config["gn"].as<Vector3d>());

            // GPS Parameters

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
