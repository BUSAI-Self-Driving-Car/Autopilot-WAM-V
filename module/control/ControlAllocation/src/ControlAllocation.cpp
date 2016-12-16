#include "ControlAllocation.h"

#include "extension/Configuration.h"
#include "utility/convert/yaml-eigen.h"
#include "message/control/Tau.h"
#include "message/propulsion/PropulsionSetpoint.h"
#include "message/navigation/StateEstimate.h"
#include "message/control/TauOveride.h"
#include "utility/policy/VehicleState.hpp"
#include <opengnc/common/math.hpp>
#include <boost/math/special_functions/sign.hpp>


namespace module {
namespace control {

using extension::Configuration;
using message::control::Tau;
using message::propulsion::PropulsionSetpoint;
using message::navigation::StateEstimate;
using message::control::TauOveride;
using utility::policy::VehicleState;
using boost::math::sign;

ControlAllocation::ControlAllocation(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment))
    , Kfwd(0)
    , Krev(0)
    , Pfwd(0)
    , Prev(0)
    , Maxradpersecfwd(1)
    , Maxradpersecrev(1)
{
    on<Configuration>("ControlAllocation.yaml").then([this] (const Configuration& config) {

        using namespace Eigen;

        auto x0 = config["x0"].as<Vector4d>();
        auto P = config["P"].as<Matrix2d>();
        auto Q = config["Q"].as<Matrix3d>();
        auto Omega = config["Omega"].as<Matrix2d>();
        double qpIterations = config["qpIterations"];

        auto configActuatorConfig = config["ActuatorConfig"];
        QPControlAllocation::ActuatorConfig actuatorConfig;
        actuatorConfig.M1x = configActuatorConfig["M1x"];
        actuatorConfig.M1y = configActuatorConfig["M1y"];
        actuatorConfig.M2x = configActuatorConfig["M2x"];
        actuatorConfig.M2y = configActuatorConfig["M2y"];

        auto configActuatorContraints = config["ActuatorConstraints"];
        QPControlAllocation::ActuatorContraints actuatorConstraints;
        actuatorConstraints.Fmin = configActuatorContraints["Fmin"];
        actuatorConstraints.Fmax = configActuatorContraints["Fmax"];
        actuatorConstraints.alphaMin = configActuatorContraints["alphaMin"];
        actuatorConstraints.alphaMax = configActuatorContraints["alphaMax"];
        actuatorConstraints.DeltaAlphaMin = configActuatorContraints["DeltaAlphaMin"];
        actuatorConstraints.DeltaAlphaMax = configActuatorContraints["DeltaAlphaMax"];

        auto configPropModel = config["PropModel"];
        Kfwd = configPropModel["Kfwd"];
        Krev = configPropModel["Krev"];
        Pfwd = configPropModel["Pfwd"];
        Prev = configPropModel["Prev"];

        Maxradpersecfwd = configPropModel["Maxradpersecfwd"];
        Maxradpersecrev = configPropModel["Maxradpersecrev"];

        qpControlAllocation.init(x0, P, Q, Omega, actuatorConfig, actuatorConstraints, qpIterations);

        auto tauOveride = std::make_unique<TauOveride>();
        tauOveride->value << 0,0,0;
        tauOveride->overide_surge = false;
        tauOveride->overide_sway = false;
        tauOveride->overide_yaw = false;
        emit(tauOveride);

        log<NUClear::INFO>("Initialised");
    });

    on<Network<Tau>>().then([this] (const Tau& tau) { emit(std::make_unique<Tau>(tau)); });
    on<Trigger<Tau>, With<StateEstimate>, With<PropulsionSetpoint>, With<TauOveride>>()
            .then([this] (
                  const Tau& tau,
                  const StateEstimate& state,
                  const PropulsionSetpoint& props,
                  const TauOveride& tauOveride) {

        if (qpControlAllocation.initialised()) {

            Eigen::Vector3d tauDesired = tau.value;
            if (tauOveride.overide_surge) {
                tauDesired[0] = tauOveride.value[0];
            }
            if (tauOveride.overide_sway){
                tauDesired[1] = tauOveride.value[1];
            }
            if (tauOveride.overide_yaw) {
                tauDesired[2] = tauOveride.value[2];
            }

            Eigen::Vector4d cmd = qpControlAllocation(tau.value);

            VehicleState statePolicy;
            Eigen::Vector3d vBNb = statePolicy.vBNb(state.x);

            Eigen::Vector3d thetatb_port;
            thetatb_port << 0,0, -props.port.azimuth;
            auto u_port = opengnc::common::math::rotationEuler(thetatb_port)*vBNb;

            Eigen::Vector3d thetatb_starbord;
            thetatb_starbord << 0,0, -props.port.azimuth;
            auto u_starboard = opengnc::common::math::rotationEuler(thetatb_starbord)*vBNb;

            auto setpoint = std::make_unique<PropulsionSetpoint>();
            setpoint->port.throttle = force2Torqueedo(cmd[0], u_port[0]);
            setpoint->port.azimuth = cmd[2];
            setpoint->starboard.throttle = force2Torqueedo(cmd[1], u_starboard[0]);
            setpoint->starboard.azimuth = cmd[3];

            emit<Scope::LOCAL, Scope::NETWORK>(setpoint);

            log( "Port throttle:", setpoint->port.throttle,
                 ", azimuth:",
                 setpoint->port.azimuth,
                 ". Starboard throttle:",
                 setpoint->starboard.throttle,
                 ", azimuth:",
                 setpoint->starboard.azimuth);
        }
    });
}

double ControlAllocation::force2Torqueedo(double F, double u)
{
    double K = Kfwd;
    double P = Pfwd;
    double radpersecmax = Maxradpersecfwd;

    if (F < 0) {
        K = Krev;
        P = Prev;
        radpersecmax = Maxradpersecrev;
    }

    double radpersec = sign(F) * (u + std::sqrt(std::abs(2*F)/K))*(2*M_PI)/P;

    return radpersec/radpersecmax;
}

}
}
