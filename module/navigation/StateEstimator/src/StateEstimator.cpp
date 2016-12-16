#include "StateEstimator.h"

#include "utility/convert/yaml-eigen.h"
#include "extension/Configuration.h"
#include "message/sensor/IMURaw.h"
#include "message/sensor/GPSRaw.h"
#include "message/navigation/StateEstimate.h"
#include <opengnc/common/transforms/wgs84.hpp>
#include "utility/Clock.h"

namespace module {
namespace navigation {

    using extension::Configuration;
    using message::sensor::IMURaw;
    using message::sensor::GPSRaw;
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
            imuVarianceDiag = config["imu_variance_diag"].as<Matrix<double,9,1>>();

            // GPS Parameters
            using namespace opengnc::common::transforms;
            Vector3d gpsOrigin = config["gps_origin"].as<Vector3d>();
            auto rNOe = wgs84::geodetic_to_rectangular(gpsOrigin);
            auto Ren = wgs84::Ren_from_geodetic(gpsOrigin);
            gpsMeasurementModel.initialise(rNOe, Ren);
            gpsVarianceDiag = config["gps_variance_diag"].as<Vector3d>();
            lagTolerance = config["lag_tolerance"].as<unsigned int>();

            // Initialise State Density
            x0 = config["x_init"].as<Matrix<double,16,1>>();
            P0_diag = config["P_init_diag"].as<Matrix<double,16,1>>();
            stateDensity = StateDensity(x0,  P0_diag.asDiagonal());
            lastUpdatedms = utility::Clock::ToMilli(NUClear::clock::now());

            emitState();

        });

        on<Trigger<IMURaw>, Sync<StateEstimator>>()
        .then("IMU Measurement", [this] (const IMURaw& msg) {

            auto timestamp = utility::Clock::ToMilli(msg.timestamp);

            int lag = lastUpdatedms - timestamp;
            if (lag < 0) {
                double timestep = -static_cast<double>(lag) / 1000;
                timeUpdate(timestep);
            }

            lastUpdatedms = timestamp;

            Eigen::Vector3d mag =  msg.magnetometer.cast<double>();
            mag.normalize();

            IMUDensity::vec_type y;
            y << msg.accelerometer.cast<double>(),
                    msg.gyroscope.cast<double>(),
                    msg.magnetometer.cast<double>();

            IMUDensity::mat_type Py = imuVarianceDiag.asDiagonal();

            IMUDensity imuDensity(y, Py);

            imuMeasurementUpdate(stateDensity, imuDensity);
            emitState();

        });

        on<Trigger<GPSRaw>, Sync<StateEstimator>>()
        .then("GPS Measurement", [this] (const GPSRaw& msg) {

            if (msg.fix_type < GPSRaw::FixType::GPS_FIX) return;

            auto timestamp = utility::Clock::ToMilli(msg.timestamp);

            int lag = lastUpdatedms - timestamp;
            if (lag < 0) {
                double timestep = -static_cast<double>(lag) / 1000;
                timeUpdate(timestep);
            }

            lastUpdatedms = timestamp;

            GPSDensity::vec_type y;
            y << msg.lla;

            GPSDensity::mat_type Py = gpsVarianceDiag.asDiagonal();

            GPSDensity gpsDensity(y, Py);
            gpsMeasurementUpdate(stateDensity, gpsDensity);

            emitState();

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
        emit<Scope::LOCAL, Scope::NETWORK>(msg, "", true);
    }
}
}
