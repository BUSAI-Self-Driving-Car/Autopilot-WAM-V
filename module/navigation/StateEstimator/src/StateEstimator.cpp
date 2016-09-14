#include "StateEstimator.h"

#include "utility/convert/yaml-eigen.h"
#include "extension/Configuration.h"
#include "message/sensor/IMU.h"
#include "message/sensor/GPS.h"
#include "message/navigation/StateEstimate.h"
#include <opengnc/common/transforms/wgs84.hpp>
#include "utility/Clock.h"

namespace module {
namespace navigation {

    using extension::Configuration;
    using message::sensor::IMU;
    using message::sensor::GPS;
    using message::navigation::StateEstimate;

    StateEstimator::StateEstimator(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , processUpdate(processModel)
        , imuMeasurementUpdate(imuMeasurementModel)
        , gpsMeasurementUpdate(gpsMeasurementModel)
        , lastUpdatedms(0)
        , lagTolerance(0)
    {

        on<Configuration>("StateEstimator.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file StateEstimator.yaml

            using namespace Eigen;
            // IMU Parameters

            imuMeasurementModel.set_mag_scale(config["mag_scale"].as<double>());
            imuMeasurementModel.set_mag_vector(config["mag_vector"].as<Vector3d>());
            imuMeasurementModel.set_Rib(config["mag_vector"].as<Matrix3d>());
            imuMeasurementModel.set_rIBb(config["rIBb"].as<Vector3d>());
            imuMeasurementModel.set_gn(config["gn"].as<Vector3d>());

            // GPS Parameters
            using namespace opengnc::common::transforms;
            Vector3d gpsOrigin = config["gps_origin"].as<Vector3d>();
            auto rNOe = wgs84::geodetic_to_rectangular(gpsOrigin);
            auto Ren = wgs84::Ren_from_geodetic(gpsOrigin);
            gpsMeasurementModel.initialise(rNOe, Ren);

            lagTolerance = config["lag_tolerance"].as<unsigned int>();

            //TODO: Initialise State Density

            lastUpdatedms = utility::Clock::ToMilli(NUClear::clock::now());
        });

        on<Trigger<IMU>, Sync<StateEstimator>>()
        .then("IMU Measurement", [this] (const IMU& meas) {
            auto timestamp = utility::Clock::ToMilli(meas.timestamp);
            int lag = lastUpdatedms - timestamp;

            if (lag < lagTolerance) {

                if (lag < 0) {
                    double timestep = static_cast<double>(lag) / 1000;
                    timeUpdate(timestep);
                }

                lastUpdatedms = timestamp;

                IMUDensity imuDensity(meas.y,meas.Py);
                imuMeasurementUpdate(stateDensity,imuDensity);

                emitState();
            }
        });

        on<Trigger<GPS>, Sync<StateEstimator>>()
        .then("GPS Measurement", [this] (const GPS& meas) {
            auto timestamp = utility::Clock::ToMilli(meas.timestamp);
            int lag = lastUpdatedms - timestamp;

            if (lag < lagTolerance) {

                if (lag < 0) {
                    double timeStep = static_cast<double>(lag) / 1000;
                    timeUpdate(timestamp);
                }

                lastUpdatedms = timestamp;

                GPSDensity imuDensity(meas.y,meas.Py);
                gpsMeasurementUpdate(stateDensity,imuDensity);

                emitState();
            }
        });
    }

    void StateEstimator::timeUpdate(double timestep)
    {
        processModel.set_timestep(timestep);
        processUpdate(stateDensity);
    }

    void StateEstimator::emitState()
    {      
        auto msg = std::make_unique<StateEstimate>();
        msg->x = stateDensity.mean();
        msg->Px = stateDensity.covariance();
        emit(msg);
    }
}
}
