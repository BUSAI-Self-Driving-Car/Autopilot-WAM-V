#ifndef MODULE_NAVIGATION_STATEESTIMATOR_H
#define MODULE_NAVIGATION_STATEESTIMATOR_H

#include <nuclear>
#include <opengnc/common/math.hpp>
#include <opengnc/common/first_order_density.hpp>
#include <opengnc/estimation/unscented_transform.hpp>
#include <opengnc/estimation/time_update.hpp>
#include <opengnc/estimation/measurement_update.hpp>
#include <opengnc/estimation/models/process/rigid_body/constant_acceleration.hpp>
#include <opengnc/estimation/models/process/rigid_body/dwna_covariance_policy.hpp>
#include <opengnc/estimation/models/measurement/imu.hpp>
#include <opengnc/estimation/models/measurement/gps.hpp>

#include "utility/policy/VehicleState.hpp"

namespace module {
namespace navigation {

    class StateEstimator : public NUClear::Reactor {

        using StatePolicy = utility::policy::VehicleState;
        using StateDensity = opengnc::common::first_order_density<double, StatePolicy::state_vector_length>;

        template<typename Model>
        using UT = opengnc::estimation::unscented_transform<Model,StatePolicy>;

        using ProcessCovariancePolicy = opengnc::estimation::models::process::rigid_body::dwna_covariance_policy<StatePolicy>;
        using ProcessModel = opengnc::estimation::models::process::rigid_body::constant_acceleration<StatePolicy,ProcessCovariancePolicy>;
        using ProcessUpdate = opengnc::estimation::time_update<StateDensity,ProcessModel,UT<ProcessModel>,StatePolicy>;

        using MeasurementModelIMU = opengnc::estimation::models::measurement::imu<StatePolicy>;
        using MeasurementModelGPS = opengnc::estimation::models::measurement::gps<StatePolicy>;

        using IMUDensity = opengnc::common::first_order_density<double, MeasurementModelIMU::output_length>;
        using GPSDensity = opengnc::common::first_order_density<double, MeasurementModelGPS::output_length>;

        template<typename MeasurementModel, typename MeasurementDensity>
        using MeasurementUpdate = opengnc::estimation::measurement_update<StateDensity,MeasurementDensity,MeasurementModel,UT<MeasurementModel>,StatePolicy>;
        using MeasurementUpdateIMU = MeasurementUpdate<MeasurementModelIMU,IMUDensity>;
        using MeasurementUpdateGPS = MeasurementUpdate<MeasurementModelGPS,GPSDensity>;

    public:
        /// @brief Called by the powerplant to build and setup the StateEstimator reactor.
        explicit StateEstimator(std::unique_ptr<NUClear::Environment> environment);

    private:
        StateDensity stateDensity;
        ProcessModel processModel;
        MeasurementModelIMU imuMeasurementModel;
        MeasurementModelGPS gpsMeasurementModel;
        ProcessUpdate processUpdate;
        MeasurementUpdateIMU imuMeasurementUpdate;
        MeasurementUpdateGPS gpsMeasurementUpdate;

    };




}
}

#endif  // MODULE_NAVIGATION_STATEESTIMATOR_H
