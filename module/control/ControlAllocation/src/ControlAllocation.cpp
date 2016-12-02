#include "ControlAllocation.h"

#include "extension/Configuration.h"
#include "utility/convert/yaml-eigen.h"
#include "message/control/Tau.h"
#include "message/propulsion/PropulsionSetpoint.h"

namespace module {
namespace control {

using extension::Configuration;
using message::control::Tau;
using message::propulsion::PropulsionSetpoint;

ControlAllocation::ControlAllocation(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

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

        qpControlAllocation.init(x0, P, Q, Omega, actuatorConfig, actuatorConstraints, qpIterations);

        log<NUClear::INFO>("Initialised");
    });

    on<Trigger<Tau>>().then([this] (const Tau& tau) {
        if (qpControlAllocation.initialised()) {

            Eigen::Vector4d cmd = qpControlAllocation(tau.value);

            auto setpoint = std::make_unique<PropulsionSetpoint>();
            setpoint->port.throttle = cmd[0];
            setpoint->port.azimuth = cmd[2];
            setpoint->starboard.throttle = cmd[1];
            setpoint->starboard.azimuth = cmd[3];

        }
    });
}
}
}
