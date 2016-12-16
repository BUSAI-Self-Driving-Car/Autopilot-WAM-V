#include "PID.h"

#include "extension/Configuration.h"
#include "message/control/PositionReference.h"
#include "message/control/VelocityCommand.h"
#include "message/control/VelocityReference.h"
#include "message/control/Tau.h"
#include "message/navigation/StateEstimate.h"
#include "utility/Clock.h"
#include "utility/policy/VehicleState.hpp"
#include "opengnc/common/math.hpp"

namespace module {
namespace control {

    using extension::Configuration;
    using message::control::PositionReference;
    using message::control::VelocityCommand;
    using message::control::VelocityReference;
    using message::control::Tau;
    using message::navigation::StateEstimate;

    PID::PID(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("PID.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file PID.yaml

            const double timestep = 0.02;
            auto func = [&] (const Configuration& root, CascadeController& controller)
            {
                {
                    const auto controller_config = root["velocity"];
                    double P = controller_config["P"];
                    double I = controller_config["I"];
                    double D = controller_config["D"];
                    double N = controller_config["N"];
                    double slew_max = controller_config["slew_max"];
                    double slew_min = controller_config["slew_min"];
                    double sat_max = controller_config["sat_max"];
                    double sat_min = controller_config["sat_min"];
                    controller.velocity.initialise(P, I, D, N, timestep, slew_max, slew_min, sat_max, sat_min);
                }

                {
                    const auto controller_config = root["position"];
                    double P = controller_config["P"];
                    double I = controller_config["I"];
                    double D = controller_config["D"];
                    double N = controller_config["N"];
                    double slew_max = controller_config["slew_max"];
                    double slew_min = controller_config["slew_min"];
                    double sat_max = controller_config["sat_max"];
                    double sat_min = controller_config["sat_min"];
                    controller.position.initialise(P, I, D, N, timestep, slew_max, slew_min, sat_max, sat_min);
                }
            };

            func(config["surge"], surge);
            func(config["sway"], sway);
            func(config["yaw"], yaw);
        });

        on<Network<VelocityReference>>().then([this](const VelocityReference& ref) { emit(std::make_unique<VelocityReference>(ref)); });
        on<Trigger<StateEstimate>, With<VelocityReference>, Optional<With<VelocityCommand>>>().then([this] (const StateEstimate& state_estimate,
                                                                                                    const VelocityReference& velocity_reference,
                                                                                                    std::shared_ptr<const VelocityCommand> velocity_command)
        {
            using utility::policy::VehicleState;
            using opengnc::common::math;

            const auto now = NUClear::clock::now();
            const double timestep = utility::Clock::ToMilli(now - last_velocity);

            auto tau = std::make_unique<Tau>();

            // Surge
            {
                double vel_ref = velocity_reference.value[0];
                if (velocity_command) { vel_ref += velocity_command->value[0]; }
                tau->value[0] = surge.velocity(vel_ref, VehicleState::vBNb(state_estimate.x)[0], timestep);
            }

            // Sway
            {
                double vel_ref = velocity_reference.value[1];
                if (velocity_command) { vel_ref += velocity_command->value[1]; }
                tau->value[1] =  sway.velocity(vel_ref, VehicleState::vBNb(state_estimate.x)[1], timestep);
            }

            // Yaw
            {
                double vel_ref = velocity_reference.value[2];
                if (velocity_command) { vel_ref += velocity_command->value[2]; }
                tau->value[2] = yaw.velocity(vel_ref, VehicleState::omegaBNb(state_estimate.x)[2], timestep);
            }

            emit(tau);
            last_velocity = now;
        });

        on<Trigger<StateEstimate>, With<PositionReference>>().then([this] (const StateEstimate& state_estimate,
                                                                   const PositionReference& position_reference)
        {
            using utility::policy::VehicleState;
            using opengnc::common::math;

            const auto now = NUClear::clock::now();
            const double timestep = utility::Clock::ToMilli(now - last_position);

            auto velocity_command = std::make_unique<VelocityCommand>();

            auto Rnb = math::rotationQuaternion(VehicleState::thetanb(state_estimate.x));
            Eigen::Vector3d ned_velocity(0,0,0);

            // North
            ned_velocity[0] = surge.position(position_reference.value[0], VehicleState::rBNn(state_estimate.x)[0], timestep);

            // East
            ned_velocity[1] = sway.position(position_reference.value[1], VehicleState::rBNn(state_estimate.x)[1], timestep);

            // Yaw
            velocity_command->value[2]= yaw.position(position_reference.value[2], math::eulerRotation(Rnb)[2], timestep);

            const Eigen::Vector3d body_velocity = Rnb.transpose() * ned_velocity;

            // Ignore any down component
            velocity_command->value[0] = body_velocity[0];
            velocity_command->value[1] = body_velocity[1];

            emit(velocity_command);
            last_position = now;
        });
    }
}
}
