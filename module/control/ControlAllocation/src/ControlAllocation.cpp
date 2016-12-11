#include "ControlAllocation.h"

#include "extension/Configuration.h"
#include "utility/convert/yaml-eigen.h"
#include "message/control/Tau.h"
#include "message/propulsion/PropulsionSetpoint.h"
#include "message/navigation/StateEstimate.h"
#include "utility/policy/VehicleState.hpp"
#include <opengnc/common/math.hpp>
#include <boost/math/special_functions/sign.hpp>


namespace module {
namespace control {

using extension::Configuration;
using message::control::Tau;
using message::propulsion::PropulsionSetpoint;
using message::navigation::StateEstimate;
using utility::policy::VehicleState;
using boost::math::sign;

ControlAllocation::ControlAllocation(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment))
    , Kfwd(0)
    , Krev(0)
    , Pfwd(0)
    , Prev(0)
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

        qpControlAllocation.init(x0, P, Q, Omega, actuatorConfig, actuatorConstraints, qpIterations);

        log<NUClear::INFO>("Initialised");
    });

    on<Trigger<Tau>, With<StateEstimate>, With<PropulsionSetpoint>>().then([this] (const Tau& tau, const StateEstimate& state, const PropulsionSetpoint& props) {
        if (qpControlAllocation.initialised()) {

            Eigen::Vector4d cmd = qpControlAllocation(tau.value);

            VehicleState statePolicy;

            Eigen::Vector3d thetatb_port;
            thetatb_port << 0,0, -props.port.azimuth;
            auto u_port = opengnc::common::math::rotationEuler(thetatb_port)*statePolicy.vBNb(state.x);

            Eigen::Vector3d thetatb_starbord;
            thetatb_starbord << 0,0, -props.port.azimuth;
            auto u_starboard = opengnc::common::math::rotationEuler(thetatb_starbord)*statePolicy.vBNb(state.x);

            auto setpoint = std::make_unique<PropulsionSetpoint>();
            setpoint->port.throttle = force2Torqueedo(cmd[0], u_port[0]);
            setpoint->port.azimuth = cmd[2];
            setpoint->starboard.throttle = force2Torqueedo(cmd[1], u_starboard[0]);
            setpoint->starboard.azimuth = cmd[3];

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
