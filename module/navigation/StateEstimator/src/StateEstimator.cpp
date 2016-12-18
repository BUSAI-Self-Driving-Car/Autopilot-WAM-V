#include "StateEstimator.h"

#include "utility/convert/yaml-eigen.h"
#include "extension/Configuration.h"
#include "message/sensor/IMURaw.h"
#include "message/sensor/GPSRaw.h"
#include "message/navigation/StateEstimate.h"
#include "message/navigation/BoatState.h"
#include <opengnc/common/transforms/wgs84.hpp>
#include "utility/Clock.h"

namespace module {
namespace navigation {

    using extension::Configuration;
    using message::sensor::IMURaw;
    using message::sensor::GPSRaw;
    using message::navigation::StateEstimate;
    using message::navigation::BoatState;

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

            Rnmag = config["may_pca_trans"].as<Matrix3d>();
            Matrix3d Roll180;
            Roll180 << 1,  0,  0,
                       0, -1,  0,
                       0,  0, -1;
            //Rnmag = Roll180*Rnmag;
            magOffset = config["mag_offset"].as<Vector3d>();


            imuMeasurementModel.set_mag_scale(config["mag_scale"].as<double>());
            imuMeasurementModel.set_mag_vector(config["mag_vector"].as<Vector3d>());
            imuMeasurementModel.set_Rib(config["Rib"].as<Matrix3d>());
            imuMeasurementModel.set_rIBb(config["rIBb"].as<Vector3d>());
            imuMeasurementModel.set_gn(config["gn"].as<Vector3d>());
            imuMeasurementModel.set_Rnmag(Rnmag);
            imuVarianceDiag = config["imu_variance_diag"].as<Matrix<double,9,1>>();



            log("IMU mag_scale", config["mag_scale"].as<double>());
            log("IMU mag_vector", config["mag_vector"].as<Vector3d>().transpose());
            log("IMU Rib", config["Rib"].as<Matrix3d>());
            log("IMU rIBb", config["rIBb"].as<Vector3d>().transpose());
            log("IMU gn", config["gn"].as<Vector3d>().transpose());
            log("IMU Variance", imuVarianceDiag.transpose());
            log("IMU Rnmag", Rnmag);


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
//log("IMU");
            auto timestamp = utility::Clock::ToMilli(msg.timestamp);

            int lag = lastUpdatedms - timestamp;
            if (lag < 0) {
                double timestep = -static_cast<double>(lag) / 1000;
                timeUpdate(timestep);
            }

            lastUpdatedms = timestamp;

            Eigen::Vector3d mag =  msg.magnetometer.cast<double>();
            mag = Rnmag*mag - magOffset;
            mag[2] = 0;

            IMUDensity::vec_type y;
            y << msg.accelerometer.cast<double>(),
                    msg.gyroscope.cast<double>(),
                    mag;

            IMUDensity::mat_type Py = imuVarianceDiag.asDiagonal();

            IMUDensity imuDensity(y, Py);
//log("meas", y.transpose());


            imuMeasurementUpdate(stateDensity, imuDensity);

           log("Pred", imuMeasurementUpdate.predicted_measurements().transpose());
            emitState();

        });

        on<Trigger<GPSRaw>, Sync<StateEstimator>>()
        .then("GPS Measurement", [this] (const GPSRaw& msg) {
//log("GPS");
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
        emit(msg);

        auto& x = stateDensity.mean();
        Eigen::Matrix<double, 16, 1> Px = stateDensity.covariance().diagonal();

        auto boatState = std::make_unique<BoatState>();

        Eigen::Vector3d theta =  opengnc::common::math::eulerRotation(StatePolicy::Rnb(x)) * 180.0/M_PI;

        boatState->north = x[0];
        boatState->east =  x[1];
        boatState->down =  x[2];
        boatState->roll =  theta[0];
        boatState->pitch =  theta[1];
        boatState->yaw =  theta[2];
        boatState->surge_vel =  x[7];
        boatState->sway_vel =  x[8];
        boatState->heave_vel =  x[9];
        boatState->roll_rate =  x[10];
        boatState->pitch_rate =  x[11];
        boatState->yaw_rate =  x[12];
        boatState->gyro_bias_r = x[13];
        boatState->gyro_bias_p =  x[14];
        boatState->gyro_bias_y =  x[15];

        boatState->Pnorth =  Px[0];
        boatState->Peast =  Px[1];
        boatState->Pdown =  Px[2];
        boatState->Pq1 =  Px[3];
        boatState->Pq2 =  Px[4];
        boatState->Pq3 =  Px[5];
        boatState->Pq4 =  Px[6];
        boatState->Psurge_vel = Px[7];
        boatState->Psway_vel = Px[8];
        boatState->Pheave_vel = Px[9];
        boatState->Proll_rate = Px[10];
        boatState->Ppitch_rate = Px[11];
        boatState->Pyaw_rate = Px[12];
        boatState->Pgyro_bias_r = Px[13];
        boatState->Pgyro_bias_p = Px[14];
        boatState->Pgyro_bias_y = Px[15];


        emit<Scope::NETWORK, Scope::LOCAL>(boatState, "", true);

        //log("Update");
    }
}
}
