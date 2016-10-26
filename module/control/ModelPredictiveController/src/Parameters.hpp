#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

struct Parameters {
    int HORIZON;
    int STATE_DIM;
    int INPUT_DIM;
    double tau;			// control time interval

    // Weightings
    double Q_N;
    double Q_E;
    double Q_psi;
    double Q_u;
    double Q_v;
    double Q_r;

    double R_angle;
    double R_thrust;

    double dR_angle;
    double dR_thrust;

    // Convergence
    double tol_angle;
    double tol_thrust;
    double tol_N;
    double tol_E;
    double tol_psi;
    double tol_u;
    double tol_v;
    double tol_r;

    double tol_rel;

    double MAX_SQP_ITERS;

    // Linesearch params
    double LS_p;
    double LS_c;

    // Actuator params
    double angle_max_change; // per sec
    double angle_max;
    double angle_min;

    double thrust_max_change; // per sec
    double thrust_max;
    double thrust_min;

    double trust_angle;
    double trust_thrust;

    // Model Params; see model for more details
    double ly;		// distance to thrusters from body fixed co-ord, along y axis
    double lx;			// distance to thrusters from body fixed co-ord, along x axis
    double m;			// boat mass
    double Iz;			// rigid body inertia
    double xg;			// distance to c.g. from body fixed co-ord, along x axis; assumed symmetrical boat
    double Xudot;		// added surge mass
    double Yvdot;      // added sway mass
    double Nrdot;		// added inertia
    double Yrdot;			// off diagonal added mass
    double Nvdot;			// off diagonal added mass
    double Xu;			// linear surge drag
    double Yv;			// linear sway drag
    double Nr;			// linear yaw drag
    double Yr;			// linear sway drag due to rotation
    double Nv;			// linear yaw drag due to sway
    //double Xuu -268		// quadratic surge drag
    double Yvv;		// quadratic sway drag
    double Nrr;			// quadratic yaw drag
    double Yrv;			// additional quadratic drags
    double Yvr;			//			''
    double Yrr;			//			''
    double Nvv;			//			..
    double Nrv;		//			..
    double Nvr;
};


#define AUTO_PARMS \
   auto& HORIZON = p.HORIZON; \
   auto& STATE_DIM = p.STATE_DIM; \
   auto& INPUT_DIM = p.INPUT_DIM; \
   auto& tau = p.tau; \
   auto& Q_N = p.Q_N; \
   auto& Q_E = p.Q_E; \
   auto& Q_psi = p.Q_psi; \
   auto& Q_u = p.Q_u; \
   auto& Q_v = p.Q_v; \
   auto& Q_r = p.Q_r; \
   auto& R_angle = p.R_angle; \
   auto& R_thrust = p.R_thrust; \
   auto& dR_angle = p.dR_angle; \
   auto& dR_thrust = p.dR_thrust; \
   auto& tol_angle = p.tol_angle; \
   auto& tol_thrust = p.tol_thrust; \
   auto& tol_N = p.tol_N; \
   auto& tol_E = p.tol_E; \
   auto& tol_psi = p.tol_psi; \
   auto& tol_u = p.tol_u; \
   auto& tol_v = p.tol_v; \
   auto& tol_r = p.tol_r; \
   auto& tol_rel = p.tol_rel; \
   auto& MAX_SQP_ITERS = p.MAX_SQP_ITERS; \
   auto& LS_p = p.LS_p; \
   auto& LS_c = p.LS_c; \
   auto& angle_max_change = p.angle_max_change; \
   auto& angle_max = p.angle_max; \
   auto& angle_min = p.angle_min; \
   auto& thrust_max_change = p.thrust_max_change; \
   auto& thrust_max = p.thrust_max; \
   auto& thrust_min = p.thrust_min; \
   auto& trust_angle = p.trust_angle; \
   auto& trust_thrust = p.trust_thrust; \
   auto& ly = p.ly; \
   auto& lx = p.lx;	 \
   auto& m = p.m; \
   auto& Iz = p.Iz; \
   auto& xg = p.xg;	 \
   auto& Xudot = p.Xudot; \
   auto& Yvdot = p.Yvdot; \
   auto& Nrdot = p.Nrdot; \
   auto& Yrdot = p.Yrdot; \
   auto& Nvdot = p.Nvdot; \
   auto& Xu = p.Xu;	 \
   auto& Yv = p.Yv;	 \
   auto& Nr = p.Nr;	 \
   auto& Yr = p.Yr;	 \
   auto& Nv = p.Nv;	 \
   auto& Yvv = p.Yvv; \
   auto& Nrr = p.Nrr; \
   auto& Yrv = p.Yrv; \
   auto& Yvr = p.Yvr; \
   auto& Yrr = p.Yrr; \
   auto& Nvv = p.Nvv; \
   auto& Nrv = p.Nrv; \
   auto& Nvr = p.Nvr; \

#endif //PARAMETERS_HPP
