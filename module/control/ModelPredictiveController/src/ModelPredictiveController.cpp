#include "ModelPredictiveController.h"

#include "extension/Configuration.h"
#include "Parameters.hpp"

namespace module {
namespace control {

    using extension::Configuration;

    ModelPredictiveController::ModelPredictiveController(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("ModelPredictiveController.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file ModelPredictiveController.yaml
            
            //TODO Populate parameters from config
            Parameters p;
            p.HORIZON = config["HORIZON"];
            p.STATE_DIM = config["STATE_DIM"];
            p.INPUT_DIM = config["INPUT_DIM"];
            p.tau = config["tau"];
            p.Q_N = config["Q_N"];
            p.Q_E = config["Q_E"];
            p.Q_psi = config["Q_psi"];
            p.Q_u = config["Q_u"];
            p.Q_v = config["Q_v"];
            p.Q_r = config["Q_r"];
            p.R_angle = config["R_angle"];
            p.R_thrust = config["R_thrust"];
            p.dR_angle = config["dR_angle"];
            p.dR_thrust = config["dR_thrust"];
            p.tol_angle = config["tol_angle"];
            p.tol_thrust = config["tol_thrust"];
            p.tol_N = config["tol_N"];
            p.tol_E = config["tol_E"];
            p.tol_psi = config["tol_psi"];
            p.tol_u = config["tol_u"];
            p.tol_v = config["tol_v"];
            p.tol_r = config["tol_r"];
            p.tol_rel = config["tol_rel"];
            p.MAX_SQP_ITERS = config["MAX_SQP_ITERS"];
            p.LS_p = config["LS_p"];
            p.LS_c = config["LS_c"];
            p.angle_max_change = config["angle_max_change"];
            p.angle_max = config["angle_max"];
            p.angle_min = config["angle_min"];
            p.thrust_max_change = config["thrust_max_change"];
            p.thrust_max = config["thrust_max"];
            p.thrust_min = config["thrust_min"];
            p.trust_angle = config["trust_angle"];
            p.trust_thrust = config["trust_thrust"];
            p.ly = config["ly"];
            p.lx = config["lx"];
            p.m = config["m"];
            p.Iz = config["Iz"];
            p.xg = config["xg"];
            p.Xudot = config["Xudot"];
            p.Yvdot = config["Yvdot"];
            p.Nrdot = config["Nrdot"];
            p.Yrdot = config["Yrdot"];
            p.Nvdot = config["Nvdot"];
            p.Xu = config["Xu"];
            p.Yv = config["Yv"];
            p.Nr = config["Nr"];
            p.Yr = config["Yr"];
            p.Nv = config["Nv"];
            p.Yvv = config["Yvv"];
            p.Nrr = config["Nrr"];
            p.Yrv = config["Yrv"];
            p.Yvr = config["Yvr"];
            p.Yrr = config["Yrr"];
            p.Nvv = config["Nvv"];
            p.Nrv = config["Nrv"];
            p.Nvr = config["Nvr"];

            //Uncomment when config is populated
            //controller.setParameters(p);
            
        });
    }
}
}
