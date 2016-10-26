#include "MPCController.h"
#include <math.h>
#include <Eigen/Dense>
#include "GradientJacobian.h"
#include "QuadraticProgram.h"

using namespace Eigen;
using Vector6d = Matrix<double,6,1>;

MPCController::MPCController()
{   
    p.HORIZON = 40;
    p.STATE_DIM = 6;
    p.INPUT_DIM = 4;
    p.tau = 0.25;			// control time interval

    // Weightings
    p.Q_N = 1;
    p.Q_E = 1;
    p.Q_psi = 1e-1;
    p.Q_u = 1e-6;
    p.Q_v = 1e-6;
    p.Q_r = 1e-6;

    p.R_angle = 1e-3;
    p.R_thrust = 1e-6;

    p.dR_angle = 1e1;
    p.dR_thrust = 1e-5;

    // Convergence
    p.tol_angle = 0.5*M_PI/180.0;
    p.tol_thrust = 5.0;
    p.tol_N = 0.05;
    p.tol_E = 0.05;
    p.tol_psi = 2*M_PI/180.0;
    p.tol_u = 0.05;
    p.tol_v = 0.05;
    p.tol_r = 2*M_PI/180.0;

    p.tol_rel = -0.5e-2;

    p.MAX_SQP_ITERS = 50;

    // Linesearch params
    p.LS_p = 1e-4;
    p.LS_c = 0.7;

    // Actuator params
    p.angle_max_change = 50.0*M_PI/180.0; // per sec
    p.angle_max = 50.0*M_PI/180.0;
    p.angle_min = -50.0*M_PI/180.0;

    p.thrust_max_change = 500; // per sec
    p.thrust_max = 500;
    p.thrust_min = -250;

    p.trust_angle = 5.0*M_PI/180.0;
    p.trust_thrust = 50.0;

    // Model Params; see model for more details
    p.ly  = 0.36;		// distance to thrusters from body fixed co-ord, along y axis
    p.lx  = 0.6;			// distance to thrusters from body fixed co-ord, along x axis
    p.m = 140.0;			// boat mass
    p.Iz = 35.0;			// rigid body inertia
    p.xg = 0.13;			// distance to c.g. from body fixed co-ord, along x axis; assumed symmetrical boat
    p.Xudot = -150;		// added surge mass
    p.Yvdot = -264;      // added sway mass
    p.Nrdot = -97;		// added inertia
    p.Yrdot = 0;			// off diagonal added mass
    p.Nvdot = 0;			// off diagonal added mass
    p.Xu = -95;			// linear surge drag
    p.Yv = -613;			// linear sway drag
    p.Nr = -105;			// linear yaw drag
    p.Yr = 0;			// linear sway drag due to rotation
    p.Nv = 0;			// linear yaw drag due to sway
    //Xuu -268		// quadratic surge drag
    p.Yvv = -164;		// quadratic sway drag
    p.Nrr = -13;			// quadratic yaw drag
    p.Yrv = 0;			// additional quadratic drags
    p.Yvr = 0;			//			''
    p.Yrr = 0;			//			''
    p.Nvv = 0;			//			..
    p.Nrv = 0;			//			..
    p.Nvr = 0;
}

MPCController::MPCController(Parameters parameters)
    : p(parameters)
{  }

void MPCController::setParameters(Parameters parameters) { p = parameters; }


void MPCController::operator()(double* x0, double* xref, double* u_in_p, double* uprev, double* u_out_p, double* param_est)
{
    Map<Matrix<double,Dynamic,Dynamic>> u(u_out_p,p.INPUT_DIM,p.HORIZON);
    Map<Matrix<double,Dynamic,Dynamic>> u_v(u_out_p,p.INPUT_DIM*p.HORIZON,1);
    Map<Matrix<double,Dynamic,Dynamic>> u_in(u_in_p,p.INPUT_DIM,p.HORIZON);
    MatrixXd x(p.STATE_DIM,p.HORIZON+1);
    MatrixXd xnew(p.STATE_DIM,p.HORIZON+1);
    MatrixXd unew(p.INPUT_DIM,p.HORIZON);
    MatrixXd e((p.STATE_DIM+2*p.INPUT_DIM)*p.HORIZON,1);
    VectorXd grad(p.INPUT_DIM*p.HORIZON);
    MatrixXd J(p.STATE_DIM*p.HORIZON+2*p.INPUT_DIM*p.HORIZON,p.INPUT_DIM*p.HORIZON);
    MatrixXd H(p.INPUT_DIM*p.HORIZON,p.INPUT_DIM*p.HORIZON);
    VectorXd d(p.INPUT_DIM*p.HORIZON);
    MatrixXd A(2*p.INPUT_DIM*p.HORIZON,p.INPUT_DIM*p.HORIZON);
    MatrixXd b_qp(2*p.INPUT_DIM*p.HORIZON,1);
    MatrixXd b(2*p.INPUT_DIM*p.HORIZON,1);
    MatrixXd ub_qp(p.INPUT_DIM*p.HORIZON,1);
    MatrixXd lb_qp(p.INPUT_DIM*p.HORIZON,1);

    double cost;
    double originalcost;
    int converged = 0;
    double D;
    double alpha;
    double lineiters;
    int i;
    int sqpiters = 0;

    setupConstraints(A.data(), b.data(), uprev);

    u = u_in;
    statesSim(x0,u.data(),x.data(), param_est);
    eCost(x.data(), xref, u.data(), uprev, &cost, e.data() );

    GradientJacobian grad_jac(p);
    QuadraticProgram qp(p);

    while ( (sqpiters < p.MAX_SQP_ITERS) && ( !converged ) ){
        grad_jac( x.data(), u.data(), e.data(), grad.data(), J.data(), param_est );
        H = J.transpose()*J;

        //% Setup QP constraints
        b_qp = -A*u_v+b;

        //% Add trust region without addiitional constraints
        for ( i=0 ; i < p.HORIZON; i++){
            ub_qp(p.INPUT_DIM*i) = std::min( p.angle_max - u_v(p.INPUT_DIM*i), p.trust_angle);
            ub_qp(p.INPUT_DIM*i+1) = std::min( p.angle_max - u_v(p.INPUT_DIM*i+1), p.trust_angle);
            ub_qp(p.INPUT_DIM*i+2) = std::min( p.thrust_max - u_v(p.INPUT_DIM*i+2),  p.trust_thrust);
            ub_qp(p.INPUT_DIM*i+3) = std::min( p.thrust_max - u_v(p.INPUT_DIM*i+3),  p.trust_thrust);

            lb_qp(p.INPUT_DIM*i) = std::max( p.angle_min - u_v(p.INPUT_DIM*i), - p.trust_angle);
            lb_qp(p.INPUT_DIM*i+1) = std::max( p.angle_min - u_v(p.INPUT_DIM*i+1), - p.trust_angle);
            lb_qp(p.INPUT_DIM*i+2) = std::max( p.thrust_min - u_v(p.INPUT_DIM*i+2), - p.trust_thrust);
            lb_qp(p.INPUT_DIM*i+3) = std::max( p.thrust_min - u_v(p.INPUT_DIM*i+3), - p.trust_thrust);
        }

        qp( H.data(), grad.data(), A.data(), b_qp.data(), lb_qp.data(), ub_qp.data(), d.data() );

        D = d.dot(grad);

        if (D/cost < p.tol_rel ){
            originalcost = cost;
            linesearch( &originalcost, u.data(), d.data(), &D, x0, xref, uprev, unew.data(), xnew.data(), e.data(), &cost, &alpha, &lineiters, param_est );
        } else {
            return;
        }

        converged = convergence( unew.data(), u.data(), xnew.data(), x.data() );

        u = unew;
        if ( ~converged) {
            x = xnew;
        }

        sqpiters += 1;
    }
}

void MPCController::setupConstraints(double* A_p, double* b_p, double* uprev_p)
{
    Map<Matrix<double, Dynamic, Dynamic>>A(A_p, 2*p.INPUT_DIM*p.HORIZON,p.INPUT_DIM*p.HORIZON);

    Map<Matrix<double, Dynamic, Dynamic>>b(b_p, p.INPUT_DIM,2*p.HORIZON);
    Map<Matrix<double, Dynamic, Dynamic>>uprev(uprev_p, p.INPUT_DIM,1);

    int i;
    Vector4d max_change;
    max_change << p.angle_max_change, p.angle_max_change, p.thrust_max_change, p.thrust_max_change;
    max_change = p.tau*max_change;

    A.setZero();
    for (i=0 ; i< p.HORIZON ; i++){
        A(p.INPUT_DIM*i,p.INPUT_DIM*i) = 1;
        A(p.INPUT_DIM*i+1,p.INPUT_DIM*i+1) = 1;
        A(p.INPUT_DIM*i+2,p.INPUT_DIM*i+2) = 1;
        A(p.INPUT_DIM*i+3,p.INPUT_DIM*i+3) = 1;

        A(p.INPUT_DIM*p.HORIZON+p.INPUT_DIM*i,p.INPUT_DIM*i) = -1;
        A(p.INPUT_DIM*p.HORIZON+p.INPUT_DIM*i+1,p.INPUT_DIM*i+1) = -1;
        A(p.INPUT_DIM*p.HORIZON+p.INPUT_DIM*i+2,p.INPUT_DIM*i+2) = -1;
        A(p.INPUT_DIM*p.HORIZON+p.INPUT_DIM*i+3,p.INPUT_DIM*i+3) = -1;
    }
    for (i=0 ; i< (p.HORIZON-1) ; i++){
        A(p.INPUT_DIM*(i+1),p.INPUT_DIM*i) = -1;
        A(p.INPUT_DIM*(i+1)+1,p.INPUT_DIM*i+1) = -1;
        A(p.INPUT_DIM*(i+1)+2,p.INPUT_DIM*i+2) = -1;
        A(p.INPUT_DIM*(i+1)+3,p.INPUT_DIM*i+3) = -1;

        A(p.INPUT_DIM*p.HORIZON+p.INPUT_DIM*(i+1),p.INPUT_DIM*i) = 1;
        A(p.INPUT_DIM*p.HORIZON+p.INPUT_DIM*(i+1)+1,p.INPUT_DIM*i+1) = 1;
        A(p.INPUT_DIM*p.HORIZON+p.INPUT_DIM*(i+1)+2,p.INPUT_DIM*i+2) = 1;
        A(p.INPUT_DIM*p.HORIZON+p.INPUT_DIM*(i+1)+3,p.INPUT_DIM*i+3) = 1;
    }

    for ( i=0 ; i < 2*p.HORIZON; i++){
        b.col(i) = max_change;
    }
    b.col(0) = uprev + max_change;
    b.col(p.HORIZON) = -uprev + max_change;
}

void MPCController::statesSim(double* x0, double* inputs, double* out, double* params)
{
    // butcher array
    int r = 4;
    double gamma[4] = {1.0/6.0, 1.0/3.0, 1.0/3.0, 1.0/6.0};
    double a[3][3] = {
        {	0.5,	0,		0 },
        {	0,		0.5,	0 },
        {	0,		0,		1 },
    };

    //int r = 7;
    //double gamma[7] = { 5179.0/57600.0, 0.0, 7571.0/16695.0, 393.0/640.0, -92097.0/339200.0, 187.0/2100.0, 1.0/40.0 };
    //double a[6][6] = {
    //	{	1.0/5.0,			0,					0,					0,				0,					0			},
    //	{	3.0/40,				9.0/40.0,			0,					0,				0,					0			},
    //	{	44.0/45.0,			-56.0/15.0,			32.0/9.0,			0,				0,					0			},
    //	{	19372.0/6561.0,		-25360.0/2187.0,	64448.0/6561.0,		-212.0/729.0,	0,					0			},
    //	{	9017.0/3168.0,		-355.0/33.0,		46732.0/5247.0,		49.0/176.0,		-5103.0/18656.0,	0			},
    //	{	35.0/384.0,			0,					500.0/1113.0,		125.0/192.0,	-2187.0/6784.0,		11.0/84.0	},
    //};


    int i,j,l;

    Vector6d temp;
    Vector6d xj;
    Matrix<double,6,7> kappa;
    Vector6d sum;
    Map<MatrixXd> zm(out,p.STATE_DIM,p.HORIZON+1);
    Map<Vector6d> x0m(x0,p.STATE_DIM,1);

    zm.col(0) = x0m;
    for( i = 0; i < p.HORIZON; i++){

        kappa.setZero();
        xj.setZero();
        sum.setZero();
        temp.setZero();

        for( j = 0; j < r; j++){
            temp.setZero();
            for(l=0;l<j;l++){
                temp = temp + a[j-1][l]*kappa.col(l);
            }
            xj = zm.col(i) + p.tau*temp;
            dstate(xj.data(),&inputs[4*i],kappa.col(j).data(),params);
            sum = sum + gamma[j]*kappa.col(j);
        }
        zm.col(i+1) = zm.col(i) + p.tau*sum;
    }
}

void MPCController::dstate(double* x0, double* inputs, double* out, double* params)
{
    //double	N = x0[0];
    //double	E = x0[1];
    double	psi = x0[2];
    double	u = x0[3];
    double	v = x0[4];
    double	r = x0[5];

    double	T1 = inputs[2];
    double	T2 = inputs[3];
    double	alpha1 = inputs[0];
    double	alpha2 = inputs[1];

    //double	vCnN = 0;
    //double	vCnE = 0;
    double	vCnN = params[0];
    double	vCnE = params[1];
    double  Xuu = params[2];

    double  t2 = cos(psi);
    double  t3 = sin(psi);
    double  t4 = r*r;
    double  t5 = t2*vCnN;
    double  t6 = t3*vCnE;
    double  t7 = t5+t6-u;
    double  t8 = fabs(t7);
    double  t9 = sin(alpha1);
    double  t10 = sin(alpha2);
    double  t11 = p.Yrdot*p.Yrdot;
    double  t12 = t3*vCnN;
    double  t15 = t2*vCnE;
    double  t13 = t12-t15+v;
    double  t14 = fabs(t13);
    double  t16 = cos(alpha1);
    double  t17 = cos(alpha2);
    double  t18 = fabs(r);
    double  t19 = p.m*p.m;
    double  t20 = p.xg*p.xg;
    double  t21 = t2*t2;
    double  t22 = t3*t3;
    double  t23 = vCnE*vCnE;
    double  t24 = vCnN*vCnN;
    double  t25 = p.Iz*p.Yvdot;
    double  t26 = t19*t20;
    double  t27 = p.Nrdot*p.m;
    double  t28 = t11+t25+t26+t27-p.Nrdot*p.Yvdot-p.Iz*p.m-p.Yrdot*p.m*p.xg*2.0;
    double  t29 = 1.0/t28;
    double  t30 = p.Yvdot*p.Yvdot;

    AUTO_PARMS

    out[0] = t2*u-t3*v;
    out[1] = t3*u+t2*v;
    out[2] = r;
    out[3] = -(T1*t16+T2*t17-Yrdot*t4+Xu*u+Xuu*t8*u-Yvdot*r*v-Xu*t3*vCnE-Xu*t2*vCnN+m*r*v+m*t4*xg-Xudot*r*t2*vCnE+Xudot*r*t3*vCnN+Yvdot*r*t2*vCnE-Yvdot*r*t3*vCnN-Xuu*t3*t8*vCnE-Xuu*t2*t8*vCnN)/(Xudot-m);
    out[4] = -t29*(Iz*T1*t9+Iz*T2*t10+Iz*Yr*r-Nrdot*T1*t9-Nrdot*T2*t10+Iz*Yv*v+Nr*Yrdot*r-Nrdot*Yr*r-Nrdot*Yv*v+Nv*Yrdot*v+r*t11*u+Nrdot*m*r*u-Nr*m*r*xg-Nv*m*v*xg+r*t19*t20*u+Iz*Yrr*r*t18+Iz*Xudot*r*u+Iz*Yvr*r*t14+Iz*Yrv*t18*v+Iz*Yvv*t14*v-Iz*Yv*t2*vCnE+Iz*Yv*t3*vCnN-Nrdot*Yrr*r*t18+Nrr*Yrdot*r*t18-Nrdot*Xudot*r*u-Nrdot*Yvr*r*t14+Nvr*Yrdot*r*t14-T1*Yrdot*lx*t9-T2*Yrdot*lx*t10+T1*Yrdot*ly*t16-T2*Yrdot*ly*t17-Nrdot*Yrv*t18*v+Nrv*Yrdot*t18*v-Nrdot*Yvv*t14*v+Nvv*Yrdot*t14*v+Nrdot*Yv*t2*vCnE-Nv*Yrdot*t2*vCnE-Nrdot*Yv*t3*vCnN+Nv*Yrdot*t3*vCnN-Xudot*Yrdot*u*v-Iz*m*r*u+Yrdot*Yvdot*u*v-Iz*Xudot*r*t3*vCnE-Iz*Xudot*r*t2*vCnN+Iz*Yvdot*r*t3*vCnE+Iz*Yvdot*r*t2*vCnN-Iz*Yrv*t2*t18*vCnE+Iz*Yrv*t3*t18*vCnN-Iz*Yvv*t2*t14*vCnE+Iz*Yvv*t3*t14*vCnN+Nrdot*Xudot*r*t3*vCnE+Nrdot*Xudot*r*t2*vCnN-Nrdot*Yvdot*r*t3*vCnE-Nrdot*Yvdot*r*t2*vCnN+Nrdot*Yrv*t2*t18*vCnE-Nrv*Yrdot*t2*t18*vCnE-Nrdot*Yrv*t3*t18*vCnN+Nrv*Yrdot*t3*t18*vCnN+Nrdot*Yvv*t2*t14*vCnE-Nvv*Yrdot*t2*t14*vCnE-Nrdot*Yvv*t3*t14*vCnN+Nvv*Yrdot*t3*t14*vCnN-Xudot*Yrdot*t2*t3*t23+Xudot*Yrdot*t2*t3*t24+Yrdot*Yvdot*t2*t3*t23-Yrdot*Yvdot*t2*t3*t24+Xudot*Yrdot*t2*u*vCnE-Xudot*Yrdot*t3*u*vCnN+Xudot*Yrdot*t3*v*vCnE+Xudot*Yrdot*t2*v*vCnN-Yrdot*Yvdot*t2*u*vCnE+Yrdot*Yvdot*t3*u*vCnN-Xudot*Yrdot*t21*vCnE*vCnN+Xudot*Yrdot*t22*vCnE*vCnN-Yrdot*Yvdot*t3*v*vCnE-Yrdot*Yvdot*t2*v*vCnN+Yrdot*Yvdot*t21*vCnE*vCnN-Yrdot*Yvdot*t22*vCnE*vCnN-Nrr*m*r*t18*xg-Nvr*m*r*t14*xg+T1*lx*m*t9*xg+T2*lx*m*t10*xg-T1*ly*m*t16*xg+T2*ly*m*t17*xg-Nrv*m*t18*v*xg-Nvv*m*t14*v*xg+Nv*m*t2*vCnE*xg-Nv*m*t3*vCnN*xg-Yrdot*m*r*u*xg*2.0+Xudot*m*u*v*xg-Yvdot*m*u*v*xg+Nrv*m*t2*t18*vCnE*xg-Nrv*m*t3*t18*vCnN*xg+Nvv*m*t2*t14*vCnE*xg-Nvv*m*t3*t14*vCnN*xg+Xudot*m*t2*t3*t23*xg-Xudot*m*t2*t3*t24*xg-Yvdot*m*t2*t3*t23*xg+Yvdot*m*t2*t3*t24*xg-Xudot*m*t2*u*vCnE*xg+Xudot*m*t3*u*vCnN*xg-Xudot*m*t3*v*vCnE*xg-Xudot*m*t2*v*vCnN*xg+Yvdot*m*t2*u*vCnE*xg-Yvdot*m*t3*u*vCnN*xg+Xudot*m*t21*vCnE*vCnN*xg-Xudot*m*t22*vCnE*vCnN*xg+Yvdot*m*t3*v*vCnE*xg+Yvdot*m*t2*v*vCnN*xg-Yvdot*m*t21*vCnE*vCnN*xg+Yvdot*m*t22*vCnE*vCnN*xg);
    out[5] = t29*(Nr*Yvdot*r+Nv*Yvdot*v-T1*Yrdot*t9-T2*Yrdot*t10-Yr*Yrdot*r-Yrdot*Yv*v-Nr*m*r-Nv*m*v+t30*u*v-Nrr*m*r*t18-Nvr*m*r*t14+T1*lx*m*t9+T2*lx*m*t10-T1*ly*m*t16+T2*ly*m*t17-Nrv*m*t18*v-Nvv*m*t14*v+Nv*m*t2*vCnE-Nv*m*t3*vCnN+T1*m*t9*xg+T2*m*t10*xg+Xudot*m*u*v+Yr*m*r*xg-Yvdot*m*u*v+Yv*m*v*xg+t2*t3*t23*t30-t2*t3*t24*t30-t2*t30*u*vCnE+t3*t30*u*vCnN-t3*t30*v*vCnE-t2*t30*v*vCnN+t21*t30*vCnE*vCnN-t22*t30*vCnE*vCnN+Nrr*Yvdot*r*t18+Nvr*Yvdot*r*t14-T1*Yvdot*lx*t9-T2*Yvdot*lx*t10+T1*Yvdot*ly*t16-T2*Yvdot*ly*t17+Nrv*Yvdot*t18*v+Nvv*Yvdot*t14*v-Nv*Yvdot*t2*vCnE+Nv*Yvdot*t3*vCnN-Yrdot*Yrr*r*t18-Xudot*Yrdot*r*u-Yrdot*Yvr*r*t14+Yrdot*Yvdot*r*u-Yrdot*Yrv*t18*v-Yrdot*Yvv*t14*v-Xudot*Yvdot*u*v+Yrdot*Yv*t2*vCnE-Yrdot*Yv*t3*vCnN-Nrv*Yvdot*t2*t18*vCnE+Nrv*Yvdot*t3*t18*vCnN-Nvv*Yvdot*t2*t14*vCnE+Nvv*Yvdot*t3*t14*vCnN-Xudot*Yvdot*t2*t3*t23+Xudot*Yvdot*t2*t3*t24+Xudot*Yrdot*r*t3*vCnE+Xudot*Yrdot*r*t2*vCnN-Yrdot*Yvdot*r*t3*vCnE-Yrdot*Yvdot*r*t2*vCnN+Yrdot*Yrv*t2*t18*vCnE-Yrdot*Yrv*t3*t18*vCnN+Yrdot*Yvv*t2*t14*vCnE-Yrdot*Yvv*t3*t14*vCnN+Xudot*Yvdot*t2*u*vCnE-Xudot*Yvdot*t3*u*vCnN+Xudot*Yvdot*t3*v*vCnE+Xudot*Yvdot*t2*v*vCnN-Xudot*Yvdot*t21*vCnE*vCnN+Xudot*Yvdot*t22*vCnE*vCnN+Nrv*m*t2*t18*vCnE-Nrv*m*t3*t18*vCnN+Nvv*m*t2*t14*vCnE-Nvv*m*t3*t14*vCnN+Xudot*m*t2*t3*t23-Xudot*m*t2*t3*t24-Yvdot*m*t2*t3*t23+Yvdot*m*t2*t3*t24-Xudot*m*t2*u*vCnE+Xudot*m*t3*u*vCnN+Yrr*m*r*t18*xg+Xudot*m*r*u*xg+Yvr*m*r*t14*xg-Xudot*m*t3*v*vCnE-Xudot*m*t2*v*vCnN+Yvdot*m*t2*u*vCnE-Yvdot*m*t3*u*vCnN-Yvdot*m*r*u*xg+Xudot*m*t21*vCnE*vCnN-Xudot*m*t22*vCnE*vCnN+Yvdot*m*t3*v*vCnE+Yvdot*m*t2*v*vCnN-Yvdot*m*t21*vCnE*vCnN+Yvdot*m*t22*vCnE*vCnN+Yrv*m*t18*v*xg+Yvv*m*t14*v*xg-Yv*m*t2*vCnE*xg+Yv*m*t3*vCnN*xg-Xudot*m*r*t3*vCnE*xg-Xudot*m*r*t2*vCnN*xg+Yvdot*m*r*t3*vCnE*xg+Yvdot*m*r*t2*vCnN*xg-Yrv*m*t2*t18*vCnE*xg+Yrv*m*t3*t18*vCnN*xg-Yvv*m*t2*t14*vCnE*xg+Yvv*m*t3*t14*vCnN*xg);

}

void MPCController:: eCost(double* x_pointer, double* xref_pointer, double* inputs_pointer, double* uprev_pointer, double* cost, double* e_pointer)
{
    AUTO_PARMS

    //	Map<Matrix<double,STATE_DIM,HORIZON+1>> x(x_pointer,STATE_DIM,HORIZON);//horizon wasn't + 1
    Map<MatrixXd> x(x_pointer, STATE_DIM,p.HORIZON+1);//horizon wasn't + 1

    Map<MatrixXd> xref(xref_pointer,STATE_DIM,p.HORIZON);
    Map<MatrixXd> inputs(inputs_pointer,INPUT_DIM,p.HORIZON);
    Map<Vector4d> uprev(uprev_pointer,INPUT_DIM,1);

    Map<MatrixXd> ex(e_pointer,STATE_DIM,p.HORIZON);
    Map<MatrixXd> er(e_pointer+STATE_DIM*p.HORIZON,INPUT_DIM,p.HORIZON);
    Map<MatrixXd> edr(e_pointer+STATE_DIM*p.HORIZON+INPUT_DIM*p.HORIZON,INPUT_DIM,p.HORIZON);
    //	Map<Matrix<double,(STATE_DIM+2*INPUT_DIM)*HORIZON,1>> e_v(e_pointer,STATE_DIM,HORIZON);
    Map<MatrixXd> e_v(e_pointer,(STATE_DIM+2*INPUT_DIM)*p.HORIZON,1);

    Vector6d Q;
    Vector4d R;
    Vector4d dR;

    int i;

    e_v.setZero();
    Q << Q_N, Q_E, Q_psi, Q_u, Q_v, Q_r;
    R << R_angle, R_angle, R_thrust, R_thrust;
    dR << dR_angle, dR_angle, dR_thrust, dR_thrust;
    Q = Q.cwiseSqrt();
    R = R.cwiseSqrt();
    dR = dR.cwiseSqrt();

    edr.col(0) = dR.asDiagonal()*(inputs.col(0) - uprev);
    for( i=0; i < p.HORIZON ; i++){
        ex.col(i) = Q.asDiagonal()*(x.col(i+1)-xref.col(i));
        er.col(i) = R.asDiagonal()*inputs.col(i);
        if (i){
            edr.col(i) = dR.asDiagonal()*(inputs.col(i) - inputs.col(i-1));
        }
    }

    cost[0] = 0.5*e_v.squaredNorm();
}

void MPCController::merit(double *u, double *x0, double *xref, double *uprev, double *cost, double *x, double *e, double *params)
{
    statesSim(x0,u,x,params);
    eCost(x,xref,u,uprev,cost,e);
}

void MPCController::linesearch(double *originalcost, double *u_in_p, double *d_p, double *D, double *x0, double *xref, double *uprev, double *u_out_p, double *x_out_p, double *e_out_p, double *cost, double *alpha, double *iters, double *params)
{
    static double LS_p = 1e-4;
    static double LS_c = 0.7;

    Map<MatrixXd> u_out(u_out_p,p.INPUT_DIM,p.HORIZON);
    Map<MatrixXd> u_in(u_in_p,p.INPUT_DIM,p.HORIZON);
    Map<MatrixXd> d(d_p,p.INPUT_DIM,p.HORIZON);
//	Map<Matrix<double,STATE_DIM,HORIZON>> x_out(x_out_p);
//	Map<Matrix<double,STATE_DIM*HORIZON+2*INPUT_DIM*HORIZON,1>> e_out(e_out_p);

    iters[0] = 0;
    alpha[0] = 1;
    u_out = u_in + d;

    merit(u_out_p, x0, xref, uprev, cost, x_out_p, e_out_p,params);

    while (cost[0] > ( originalcost[0] + LS_p*alpha[0]*D[0] ) ){
        iters[0] += 1;
        alpha[0] = std::min( LS_c*alpha[0], (-D[0]*alpha[0]*alpha[0])/(2*(cost[0] - originalcost[0] - D[0]*alpha[0])));
        u_out = u_in + alpha[0]*d;
        merit(u_out_p, x0, xref, uprev, cost, x_out_p, e_out_p,params);
    }

}

int MPCController::convergence(double *unew_pointer, double *u_pointer, double *xnew_pointer, double *x_pointer){

    Map<MatrixXd> x(x_pointer,p.STATE_DIM,p.HORIZON+1);
    Map<MatrixXd> u(u_pointer,p.INPUT_DIM,p.HORIZON);
    Map<MatrixXd> xnew(xnew_pointer,p.STATE_DIM,p.HORIZON+1);
    Map<MatrixXd> unew(unew_pointer,p.INPUT_DIM,p.HORIZON);

    Vector6d state_change;
    Vector4d input_change;
    Vector6d state_tol;
    Vector4d input_tol;

    int i,j;

    state_tol << p.tol_N, p.tol_E, p.tol_psi, p.tol_u, p.tol_v, p.tol_r;
    input_tol << p.tol_angle, p.tol_angle, p.tol_thrust, p.tol_thrust;

    for( i = (p.HORIZON-1) ; i >= 0 ; i--){
        state_change = (x.col(i+1) - xnew.col(i+1)).cwiseAbs();
        for(j=0;j<p.STATE_DIM;j++){
            if (state_change(j) > state_tol(j)){
                return 0;
            }
        }
    }
    if (i == -1) {
        return 1;
    }

    for( i = (p.HORIZON-1) ; i >= 0 ; i--){
        input_change = (u.col(i) - unew.col(i)).cwiseAbs();
        for(j=0;j<p.INPUT_DIM;j++){
            if (input_change(j) > input_tol(j)){
                return 0;
            }
        }
    }

    return 1;
}




