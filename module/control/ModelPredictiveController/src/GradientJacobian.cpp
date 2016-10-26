#include "GradientJacobian.h"
#include <Eigen/Dense>

using namespace Eigen;
using Vector6d = Matrix<double,6,1>;
typedef Matrix<double,6,1> Vector6d;
//typedef Matrix<double,4,1> Vector4d;
typedef Matrix<double,24,1> Vector24d;
typedef Matrix<double,36,1> Vector36d;
typedef Matrix<double,6,6> Matrix66d;
typedef Matrix<double,6,4> Matrix64d;

GradientJacobian::GradientJacobian(const Parameters& param)
    : p(param)
{ }

void GradientJacobian::operator() (double *x_pointer, double *inputs_pointer, double *e_pointer, double *grad_pointer, double *J_pointer, double *params)
{
    Map<MatrixXd> e(e_pointer,p.STATE_DIM*p.HORIZON+2*p.INPUT_DIM*p.HORIZON,1);
    Map<MatrixXd> grad(grad_pointer,p.INPUT_DIM*p.HORIZON,1);
    Map<MatrixXd> J(J_pointer,p.STATE_DIM*p.HORIZON+2*p.INPUT_DIM*p.HORIZON,p.INPUT_DIM*p.HORIZON);
    MatrixXd state_sens_matrix(p.STATE_DIM*p.HORIZON,p.INPUT_DIM*p.HORIZON);

    Vector6d Q;
    Vector4d R;
    Vector4d dR;

    int i,j;

    AUTO_PARMS

    Q << Q_N, Q_E, Q_psi, Q_u, Q_v, Q_r;
    R << R_angle, R_angle, R_thrust, R_thrust;
    dR << dR_angle, dR_angle, dR_thrust, dR_thrust;
    Q = Q.cwiseSqrt();
    R = R.cwiseSqrt();
    dR = dR.cwiseSqrt();

    J.setZero();

    // State part
    state_sens(inputs_pointer,x_pointer,state_sens_matrix.data(),params);
    for( i=0; i < p.HORIZON; i++){
        for( j = i ; j < p.HORIZON ; j++){
            J.block(p.STATE_DIM*j,p.INPUT_DIM*i,p.STATE_DIM,p.INPUT_DIM) = Q.asDiagonal()*state_sens_matrix.block(p.STATE_DIM*j,p.INPUT_DIM*i,p.STATE_DIM,p.INPUT_DIM);
        }
    }

    // Inputs
    for( i=0 ; i < p.HORIZON ; i++){
        J(p.STATE_DIM*p.HORIZON + p.INPUT_DIM*i,p.INPUT_DIM*i) = R(0);
        J(p.STATE_DIM*p.HORIZON + p.INPUT_DIM*i + 1,p.INPUT_DIM*i + 1) = R(1);
        J(p.STATE_DIM*p.HORIZON + p.INPUT_DIM*i + 2,p.INPUT_DIM*i + 2) = R(2);
        J(p.STATE_DIM*p.HORIZON + p.INPUT_DIM*i + 3,p.INPUT_DIM*i + 3) = R(3);

        J(p.STATE_DIM*p.HORIZON + p.INPUT_DIM*p.HORIZON + p.INPUT_DIM*i,p.INPUT_DIM*i) = dR(0);
        J(p.STATE_DIM*p.HORIZON + p.INPUT_DIM*p.HORIZON + p.INPUT_DIM*i + 1,p.INPUT_DIM*i + 1) = dR(1);
        J(p.STATE_DIM*p.HORIZON + p.INPUT_DIM*p.HORIZON + p.INPUT_DIM*i + 2,p.INPUT_DIM*i + 2) = dR(2);
        J(p.STATE_DIM*p.HORIZON + p.INPUT_DIM*p.HORIZON + p.INPUT_DIM*i + 3,p.INPUT_DIM*i + 3) = dR(3);
    }

    for ( i=0 ;i < (p.HORIZON-1) ; i++){
        J(p.STATE_DIM*p.HORIZON + p.INPUT_DIM*p.HORIZON + p.INPUT_DIM*(i+1),p.INPUT_DIM*i) = -dR(0);
        J(p.STATE_DIM*p.HORIZON + p.INPUT_DIM*p.HORIZON + p.INPUT_DIM*(i+1) + 1,p.INPUT_DIM*i + 1) = -dR(1);
        J(p.STATE_DIM*p.HORIZON + p.INPUT_DIM*p.HORIZON + p.INPUT_DIM*(i+1) + 2,p.INPUT_DIM*i + 2) = -dR(2);
        J(p.STATE_DIM*p.HORIZON + p.INPUT_DIM*p.HORIZON + p.INPUT_DIM*(i+1) + 3,p.INPUT_DIM*i + 3) = -dR(3);
    }

    grad = J.transpose()*e;
}

void GradientJacobian::jac_state(double *x0, double *out, double *params)
{
    AUTO_PARMS

    double	N = x0[0];
    double	E = x0[1];
    double	psi = x0[2];
    double	u = x0[3];
    double	v = x0[4];
    double	r = x0[5];

    //double	vCnN = 0;
    //double	vCnE = 0;
    double	vCnN = params[0];
    double	vCnE = params[1];
    double Xuu = params[2];

    double  t2 = cos(psi);
    double  t3 = sin(psi);
    double  t4 = t2*vCnN;
    double  t5 = t3*vCnE;
    double  t6 = t4+t5-u;
    double  t7 = fabs(t6);
    double  t8 = (t6/fabs(t6));
    double  t9 = t2*vCnE;
    double  t11 = t3*vCnN;
    double  t10 = t9-t11;
    double  t12 = Xudot-m;
    double  t13 = 1.0/t12;
    double  t14 = t2*t2;
    double  t15 = vCnE*vCnE;
    double  t16 = vCnN*vCnN;
    double  t17 = t3*t3;
    double  t18 = fabs(r);
    double  t19 = -t9+t11+v;
    double  t20 = (t19/fabs(t19));
    double  t21 = t4+t5;
    double  t22 = fabs(t19);
    double  t23 = Iz*Yvdot;
    double  t24 = m*m;
    double  t25 = xg*xg;
    double  t26 = t24*t25;
    double  t27 = Nrdot*m;
    double  t28 = Yrdot*Yrdot;
    double  t31 = Nrdot*Yvdot;
    double  t32 = Iz*m;
    double  t33 = Yrdot*m*xg*2.0;
    double  t29 = t23+t26+t27+t28-t31-t32-t33;
    double  t30 = 1.0/t29;
    double  t34 = (r/fabs(r));
    double  t35 = Yvdot*Yvdot;
    double  t36 = Yrdot*Yvdot*u;
    double  t37 = Xudot*m*u*xg;
    double  t38 = Xudot*Yrdot*t2*vCnN;
    double  t39 = Xudot*Yrdot*t3*vCnE;
    double  t40 = Yvdot*m*t2*vCnN*xg;
    double  t41 = Yvdot*m*t3*vCnE*xg;

    Map<MatrixXd> outm(out,p.STATE_DIM,p.STATE_DIM);

    outm.setZero();

    outm(0,2) = -t3*u-t2*v;
    outm(0,3) = t2;
    outm(0,4) = -t3;
    outm(1,2) = t2*u-t3*v;
    outm(1,3) = t3;
    outm(1,4) = t2;
    outm(2,5) = 1.0;
    outm(3,2) = t13*(Xu*t2*vCnE-Xu*t3*vCnN-Xudot*r*t3*vCnE-Xudot*r*t2*vCnN-Xuu*t8*t10*u+Yvdot*r*t3*vCnE+Yvdot*r*t2*vCnN+Xuu*t2*t7*vCnE-Xuu*t3*t7*vCnN+Xuu*t3*t8*t10*vCnE+Xuu*t2*t8*t10*vCnN);
    outm(3,3) = -t13*(Xu+Xuu*t7-Xuu*t8*u+Xuu*t3*t8*vCnE+Xuu*t2*t8*vCnN);
    outm(3,4) = t13*(Yvdot*r-m*r);
    outm(3,5) = t13*(Yrdot*r*2.0+Yvdot*v-m*v+Xudot*t2*vCnE-Xudot*t3*vCnN-Yvdot*t2*vCnE+Yvdot*t3*vCnN-m*r*xg*2.0);
    outm(4,2) = -t30*(Iz*Yv*t3*vCnE+Iz*Yv*t2*vCnN-Nrdot*Yv*t3*vCnE+Nv*Yrdot*t3*vCnE-Nrdot*Yv*t2*vCnN+Nv*Yrdot*t2*vCnN-Xudot*Yrdot*t14*t15+Xudot*Yrdot*t14*t16+Xudot*Yrdot*t15*t17-Xudot*Yrdot*t16*t17+Yrdot*Yvdot*t14*t15-Yrdot*Yvdot*t14*t16-Yrdot*Yvdot*t15*t17+Yrdot*Yvdot*t16*t17+Iz*Yvr*r*t20*t21-Iz*Xudot*r*t2*vCnE+Iz*Xudot*r*t3*vCnN+Iz*Yvdot*r*t2*vCnE-Iz*Yvdot*r*t3*vCnN+Iz*Yvv*t20*t21*v+Iz*Yrv*t3*t18*vCnE+Iz*Yrv*t2*t18*vCnN+Iz*Yvv*t3*t22*vCnE+Iz*Yvv*t2*t22*vCnN-Nrdot*Yvr*r*t20*t21+Nvr*Yrdot*r*t20*t21+Nrdot*Xudot*r*t2*vCnE-Nrdot*Xudot*r*t3*vCnN-Nrdot*Yvdot*r*t2*vCnE+Nrdot*Yvdot*r*t3*vCnN-Nrdot*Yvv*t20*t21*v+Nvv*Yrdot*t20*t21*v-Nrdot*Yrv*t3*t18*vCnE+Nrv*Yrdot*t3*t18*vCnE-Nrdot*Yrv*t2*t18*vCnN+Nrv*Yrdot*t2*t18*vCnN-Nrdot*Yvv*t3*t22*vCnE+Nvv*Yrdot*t3*t22*vCnE-Nrdot*Yvv*t2*t22*vCnN+Nvv*Yrdot*t2*t22*vCnN-Xudot*Yrdot*t3*u*vCnE-Xudot*Yrdot*t2*u*vCnN+Xudot*Yrdot*t2*v*vCnE-Xudot*Yrdot*t3*v*vCnN+Yrdot*Yvdot*t3*u*vCnE+Yrdot*Yvdot*t2*u*vCnN-Yrdot*Yvdot*t2*v*vCnE+Yrdot*Yvdot*t3*v*vCnN-Nv*m*t3*vCnE*xg-Nv*m*t2*vCnN*xg+Xudot*m*t14*t15*xg-Xudot*m*t14*t16*xg-Xudot*m*t15*t17*xg+Xudot*m*t16*t17*xg-Yvdot*m*t14*t15*xg+Yvdot*m*t14*t16*xg+Yvdot*m*t15*t17*xg-Yvdot*m*t16*t17*xg-Iz*Yvv*t2*t20*t21*vCnE+Iz*Yvv*t3*t20*t21*vCnN+Nrdot*Yvv*t2*t20*t21*vCnE-Nvv*Yrdot*t2*t20*t21*vCnE-Nrdot*Yvv*t3*t20*t21*vCnN+Nvv*Yrdot*t3*t20*t21*vCnN+Xudot*Yrdot*t2*t3*vCnE*vCnN*4.0-Yrdot*Yvdot*t2*t3*vCnE*vCnN*4.0-Nvr*m*r*t20*t21*xg-Nvv*m*t20*t21*v*xg-Nrv*m*t3*t18*vCnE*xg-Nrv*m*t2*t18*vCnN*xg-Nvv*m*t3*t22*vCnE*xg-Nvv*m*t2*t22*vCnN*xg+Xudot*m*t3*u*vCnE*xg+Xudot*m*t2*u*vCnN*xg-Xudot*m*t2*v*vCnE*xg+Xudot*m*t3*v*vCnN*xg-Yvdot*m*t3*u*vCnE*xg-Yvdot*m*t2*u*vCnN*xg+Yvdot*m*t2*v*vCnE*xg-Yvdot*m*t3*v*vCnN*xg+Nvv*m*t2*t20*t21*vCnE*xg-Nvv*m*t3*t20*t21*vCnN*xg-Xudot*m*t2*t3*vCnE*vCnN*xg*4.0+Yvdot*m*t2*t3*vCnE*vCnN*xg*4.0);
    outm(4,3) = -t30*(r*t28+Iz*Xudot*r-Nrdot*Xudot*r-Xudot*Yrdot*v-Iz*m*r+Yrdot*Yvdot*v+Nrdot*m*r+r*t24*t25-Yrdot*m*r*xg*2.0+Xudot*m*v*xg-Yvdot*m*v*xg+Xudot*Yrdot*t2*vCnE-Xudot*Yrdot*t3*vCnN-Yrdot*Yvdot*t2*vCnE+Yrdot*Yvdot*t3*vCnN-Xudot*m*t2*vCnE*xg+Xudot*m*t3*vCnN*xg+Yvdot*m*t2*vCnE*xg-Yvdot*m*t3*vCnN*xg);
    outm(4,4) = -t30*(t36+t37+t38+t39+t40+t41+Iz*Yv-Nrdot*Yv+Nv*Yrdot+Iz*Yrv*t18+Iz*Yvv*t22-Nrdot*Yrv*t18+Nrv*Yrdot*t18-Nrdot*Yvv*t22+Nvv*Yrdot*t22-Xudot*Yrdot*u-Nv*m*xg-Nrv*m*t18*xg-Nvv*m*t22*xg-Yvdot*m*u*xg+Iz*Yvr*r*t20+Iz*Yvv*t20*v-Nrdot*Yvr*r*t20+Nvr*Yrdot*r*t20-Nrdot*Yvv*t20*v+Nvv*Yrdot*t20*v-Yrdot*Yvdot*t3*vCnE-Yrdot*Yvdot*t2*vCnN-Iz*Yvv*t2*t20*vCnE+Iz*Yvv*t3*t20*vCnN+Nrdot*Yvv*t2*t20*vCnE-Nvv*Yrdot*t2*t20*vCnE-Nrdot*Yvv*t3*t20*vCnN+Nvv*Yrdot*t3*t20*vCnN-Nvr*m*r*t20*xg-Nvv*m*t20*v*xg-Xudot*m*t3*vCnE*xg-Xudot*m*t2*vCnN*xg+Nvv*m*t2*t20*vCnE*xg-Nvv*m*t3*t20*vCnN*xg);
    outm(4,5) = -t30*(Iz*Yr+Nr*Yrdot-Nrdot*Yr+t28*u+Iz*Yrr*t18+Iz*Xudot*u+Iz*Yvr*t22-Nrdot*Yrr*t18+Nrr*Yrdot*t18-Nrdot*Xudot*u-Nrdot*Yvr*t22+Nvr*Yrdot*t22-Iz*m*u+Nrdot*m*u-Nr*m*xg+t24*t25*u-Nrr*m*t18*xg-Nvr*m*t22*xg-Yrdot*m*u*xg*2.0+Iz*Yrr*r*t34-Iz*Xudot*t3*vCnE-Iz*Xudot*t2*vCnN+Iz*Yrv*t34*v+Iz*Yvdot*t3*vCnE+Iz*Yvdot*t2*vCnN-Nrdot*Yrr*r*t34+Nrr*Yrdot*r*t34+Nrdot*Xudot*t3*vCnE+Nrdot*Xudot*t2*vCnN-Nrdot*Yrv*t34*v+Nrv*Yrdot*t34*v-Nrdot*Yvdot*t3*vCnE-Nrdot*Yvdot*t2*vCnN-Iz*Yrv*t2*t34*vCnE+Iz*Yrv*t3*t34*vCnN+Nrdot*Yrv*t2*t34*vCnE-Nrv*Yrdot*t2*t34*vCnE-Nrdot*Yrv*t3*t34*vCnN+Nrv*Yrdot*t3*t34*vCnN-Nrr*m*r*t34*xg-Nrv*m*t34*v*xg+Nrv*m*t2*t34*vCnE*xg-Nrv*m*t3*t34*vCnN*xg);
    outm(5,2) = t30*(t14*t15*t35-t14*t16*t35-t15*t17*t35+t16*t17*t35-Nv*m*t3*vCnE-Nv*m*t2*vCnN+Xudot*m*t14*t15-Xudot*m*t14*t16-Xudot*m*t15*t17+Xudot*m*t16*t17-Yvdot*m*t14*t15+Yvdot*m*t14*t16+Yvdot*m*t15*t17-Yvdot*m*t16*t17+t3*t35*u*vCnE+t2*t35*u*vCnN-t2*t35*v*vCnE+t3*t35*v*vCnN+Nv*Yvdot*t3*vCnE+Nv*Yvdot*t2*vCnN-Xudot*Yvdot*t14*t15+Xudot*Yvdot*t14*t16+Xudot*Yvdot*t15*t17-Xudot*Yvdot*t16*t17-Yrdot*Yv*t3*vCnE-Yrdot*Yv*t2*vCnN+Nvr*Yvdot*r*t20*t21+Nvv*Yvdot*t20*t21*v+Nrv*Yvdot*t3*t18*vCnE+Nrv*Yvdot*t2*t18*vCnN+Nvv*Yvdot*t3*t22*vCnE+Nvv*Yvdot*t2*t22*vCnN-Yrdot*Yvr*r*t20*t21+Xudot*Yrdot*r*t2*vCnE-Xudot*Yrdot*r*t3*vCnN-Yrdot*Yvdot*r*t2*vCnE+Yrdot*Yvdot*r*t3*vCnN-Yrdot*Yvv*t20*t21*v-Yrdot*Yrv*t3*t18*vCnE-Yrdot*Yrv*t2*t18*vCnN-Yrdot*Yvv*t3*t22*vCnE-Yrdot*Yvv*t2*t22*vCnN-Xudot*Yvdot*t3*u*vCnE-Xudot*Yvdot*t2*u*vCnN+Xudot*Yvdot*t2*v*vCnE-Xudot*Yvdot*t3*v*vCnN-Nvr*m*r*t20*t21-Nvv*m*t20*t21*v-Nrv*m*t3*t18*vCnE-Nrv*m*t2*t18*vCnN-Nvv*m*t3*t22*vCnE-Nvv*m*t2*t22*vCnN+Xudot*m*t3*u*vCnE+Xudot*m*t2*u*vCnN-Xudot*m*t2*v*vCnE+Xudot*m*t3*v*vCnN-Yvdot*m*t3*u*vCnE-Yvdot*m*t2*u*vCnN+Yvdot*m*t2*v*vCnE-Yvdot*m*t3*v*vCnN+Yv*m*t3*vCnE*xg+Yv*m*t2*vCnN*xg-t2*t3*t35*vCnE*vCnN*4.0-Nvv*Yvdot*t2*t20*t21*vCnE+Nvv*Yvdot*t3*t20*t21*vCnN+Yrdot*Yvv*t2*t20*t21*vCnE-Yrdot*Yvv*t3*t20*t21*vCnN+Xudot*Yvdot*t2*t3*vCnE*vCnN*4.0+Nvv*m*t2*t20*t21*vCnE-Nvv*m*t3*t20*t21*vCnN+Yvr*m*r*t20*t21*xg-Xudot*m*t2*t3*vCnE*vCnN*4.0-Xudot*m*r*t2*vCnE*xg+Xudot*m*r*t3*vCnN*xg+Yvdot*m*t2*t3*vCnE*vCnN*4.0+Yvdot*m*r*t2*vCnE*xg-Yvdot*m*r*t3*vCnN*xg+Yvv*m*t20*t21*v*xg+Yrv*m*t3*t18*vCnE*xg+Yrv*m*t2*t18*vCnN*xg+Yvv*m*t3*t22*vCnE*xg+Yvv*m*t2*t22*vCnN*xg-Yvv*m*t2*t20*t21*vCnE*xg+Yvv*m*t3*t20*t21*vCnN*xg);
    outm(5,3) = t30*(t35*v-Xudot*Yrdot*r+Yrdot*Yvdot*r-Xudot*Yvdot*v+Xudot*m*v-Yvdot*m*v-t2*t35*vCnE+t3*t35*vCnN-Xudot*m*t2*vCnE+Xudot*m*t3*vCnN+Xudot*m*r*xg+Yvdot*m*t2*vCnE-Yvdot*m*t3*vCnN-Yvdot*m*r*xg+Xudot*Yvdot*t2*vCnE-Xudot*Yvdot*t3*vCnN);
    outm(5,4) = t30*(Nv*Yvdot-Yrdot*Yv-Nv*m+t35*u+Nrv*Yvdot*t18+Nvv*Yvdot*t22-Yrdot*Yrv*t18-Yrdot*Yvv*t22-Xudot*Yvdot*u-Nrv*m*t18-Nvv*m*t22+Xudot*m*u-Yvdot*m*u+Yv*m*xg-t3*t35*vCnE-t2*t35*vCnN-Nvr*m*r*t20-Nvv*m*t20*v-Xudot*m*t3*vCnE-Xudot*m*t2*vCnN+Yvdot*m*t3*vCnE+Yvdot*m*t2*vCnN+Yrv*m*t18*xg+Yvv*m*t22*xg+Nvr*Yvdot*r*t20+Nvv*Yvdot*t20*v-Yrdot*Yvr*r*t20+Xudot*Yvdot*t3*vCnE+Xudot*Yvdot*t2*vCnN-Yrdot*Yvv*t20*v-Nvv*Yvdot*t2*t20*vCnE+Nvv*Yvdot*t3*t20*vCnN+Yrdot*Yvv*t2*t20*vCnE-Yrdot*Yvv*t3*t20*vCnN+Nvv*m*t2*t20*vCnE-Nvv*m*t3*t20*vCnN+Yvr*m*r*t20*xg+Yvv*m*t20*v*xg-Yvv*m*t2*t20*vCnE*xg+Yvv*m*t3*t20*vCnN*xg);
    outm(5,5) = t30*(t36+t37+t38+t39+t40+t41+Nr*Yvdot-Yr*Yrdot-Nr*m+Nrr*Yvdot*t18+Nvr*Yvdot*t22-Yrdot*Yrr*t18-Xudot*Yrdot*u-Yrdot*Yvr*t22-Nrr*m*t18-Nvr*m*t22+Yr*m*xg-Nrr*m*r*t34-Nrv*m*t34*v+Yrr*m*t18*xg+Yvr*m*t22*xg-Yvdot*m*u*xg+Nrr*Yvdot*r*t34+Nrv*Yvdot*t34*v-Yrdot*Yrr*r*t34-Yrdot*Yrv*t34*v-Yrdot*Yvdot*t3*vCnE-Yrdot*Yvdot*t2*vCnN-Nrv*Yvdot*t2*t34*vCnE+Nrv*Yvdot*t3*t34*vCnN+Yrdot*Yrv*t2*t34*vCnE-Yrdot*Yrv*t3*t34*vCnN+Nrv*m*t2*t34*vCnE-Nrv*m*t3*t34*vCnN+Yrr*m*r*t34*xg-Xudot*m*t3*vCnE*xg-Xudot*m*t2*vCnN*xg+Yrv*m*t34*v*xg-Yrv*m*t2*t34*vCnE*xg+Yrv*m*t3*t34*vCnN*xg);
}

void GradientJacobian::wrt_inputs(double *x0, double *inputs, double *out, double *params)
{
    double	N = x0[0];
    double	E = x0[1];
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
    double Xuu = params[2];

    AUTO_PARMS

    double  t2 = Xudot-m;
    double  t3 = 1.0/t2;
    double  t4 = cos(alpha1);
    double  t5 = sin(alpha1);
    double  t6 = cos(alpha2);
    double  t7 = sin(alpha2);
    double  t8 = Iz*Yvdot;
    double  t9 = m*m;
    double  t10 = xg*xg;
    double  t11 = t9*t10;
    double  t12 = Nrdot*m;
    double  t13 = Yrdot*Yrdot;
    double  t16 = Nrdot*Yvdot;
    double  t17 = Iz*m;
    double  t18 = Yrdot*m*xg*2.0;
    double  t14 = t8+t11+t12+t13-t16-t17-t18;
    double  t15 = 1.0/t14;

    Map<MatrixXd> outm(out,p.STATE_DIM,p.INPUT_DIM);

    outm.setZero();

    outm(3,0) = T1*t3*t5;
    outm(3,1) = T2*t3*t7;
    outm(3,2) = -t3*t4;
    outm(3,3) = -t3*t6;
    outm(4,0) = -t15*(Iz*T1*t4-Nrdot*T1*t4-T1*Yrdot*lx*t4-T1*Yrdot*ly*t5+T1*lx*m*t4*xg+T1*ly*m*t5*xg);
    outm(4,1) = -t15*(Iz*T2*t6-Nrdot*T2*t6-T2*Yrdot*lx*t6+T2*Yrdot*ly*t7+T2*lx*m*t6*xg-T2*ly*m*t7*xg);
    outm(4,2) = -t15*(Iz*t5-Nrdot*t5-Yrdot*lx*t5+Yrdot*ly*t4+lx*m*t5*xg-ly*m*t4*xg);
    outm(4,3) = -t15*(Iz*t7-Nrdot*t7-Yrdot*lx*t7-Yrdot*ly*t6+lx*m*t7*xg+ly*m*t6*xg);
    outm(5,0) = -t15*(T1*Yrdot*t4-T1*lx*m*t4-T1*ly*m*t5-T1*m*t4*xg+T1*Yvdot*lx*t4+T1*Yvdot*ly*t5);
    outm(5,1) = -t15*(T2*Yrdot*t6-T2*lx*m*t6+T2*ly*m*t7-T2*m*t6*xg+T2*Yvdot*lx*t6-T2*Yvdot*ly*t7);
    outm(5,2) = -t15*(Yrdot*t5+Yvdot*lx*t5-Yvdot*ly*t4-lx*m*t5+ly*m*t4-m*t5*xg);
    outm(5,3) = -t15*(Yrdot*t7+Yvdot*lx*t7+Yvdot*ly*t6-lx*m*t7-ly*m*t6-m*t7*xg);
}

void GradientJacobian::int_Bk(double *x0, double *inputs, double *out, double *params)
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

    int j,l;

    Vector24d temp;
    Vector24d xj;
    MatrixXd kappa;
//	Map<Vector24d> sum(out,p.STATE_DIM,4); // this line fails in  controller // sum is output, should bee (6,4) ?? , was 6,,6
    Map<Vector24d> sum(out); // this line fails in  controller // sum is output, should bee (6,4) ?? , was 6,,6

//	Map<Matrix64d> sens(xj.data(),4*p.STATE_DIM,1);
    Map<Matrix64d> sens(xj.data());

    Matrix64d dx;
//	Map<Vector24d> dx_v(dx.data(),p.STATE_DIM,4);
    Map<Vector24d> dx_v(dx.data());

    Matrix64d wrtins;
    Matrix66d jac;

    // Setup j, ins
    jac_state(x0, jac.data(), params );
    wrt_inputs(x0, inputs, wrtins.data(), params);

    kappa.setZero(4*p.STATE_DIM,7);
    xj.setZero();
    sum.setZero();
    temp.setZero();
    sens.setZero();

    AUTO_PARMS

    for( j = 0; j < r; j++){
        temp.setZero();
        for(l=0;l<j;l++){
            temp = temp + a[j-1][l]*kappa.col(l);
        }
        xj = tau*temp;
        dx = jac*sens + wrtins;
        kappa.col(j) = dx_v;
        sum = sum + gamma[j]*kappa.col(j);
    }
    sum = tau*sum;
}

void GradientJacobian::int_Ak(double *x0, double *out, double *params)
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

    int j,l;

    Vector36d temp;
    Vector36d xj;
    MatrixXd kappa;
//	Map<Vector36d> sum(out,p.STATE_DIM,p.STATE_DIM); // sum is output
    Map<Vector36d> sum(out); // sum is output

//	Map<Matrix66d> sens(xj.data(),6*p.STATE_DIM,1);
    Map<Matrix66d> sens(xj.data());

    Matrix66d dx;
//	Map<Vector36d> dx_v(dx.data(),p.STATE_DIM,p.STATE_DIM);
    Map<Vector36d> dx_v(dx.data());

    Matrix66d eye6;
//	Map<Vector36d> eye6_v(eye6.data(),p.STATE_DIM,p.STATE_DIM);
    Map<Vector36d> eye6_v(eye6.data());

    Matrix66d jac;

    jac_state(x0, jac.data(), params );

    kappa.setZero(6*p.STATE_DIM,7);
    xj.setZero();
    sum.setZero();
    temp.setZero();

    eye6.setIdentity();

    AUTO_PARMS

    for( j = 0; j < r; j++){
        temp.setZero();
        for(l=0;l<j;l++){
            temp = temp + a[j-1][l]*kappa.col(l);
        }
        xj = eye6_v + tau*temp;
        dx = jac*sens;
        kappa.col(j) = dx_v;
        sum = sum + gamma[j]*kappa.col(j);
    }
    sum = eye6_v + tau*sum;
}

void GradientJacobian::state_sens(double *inputs, double *states, double *out, double *params)
{
    Map<MatrixXd> state_sens(out,p.STATE_DIM*p.HORIZON,p.INPUT_DIM*p.HORIZON);

    Matrix66d Ak;
    Matrix64d Bk;

    int i,j;

    state_sens.setZero();

    int_Bk(&states[0], &inputs[0],Bk.data(),params);
    state_sens.block(0,0,p.STATE_DIM,p.INPUT_DIM) = Bk;


    for( i = 1 ; i < p.HORIZON; i++){
        int_Bk(&states[p.STATE_DIM*i], &inputs[p.INPUT_DIM*i],Bk.data(),params);
        state_sens.block(p.STATE_DIM*i,p.INPUT_DIM*i,p.STATE_DIM,p.INPUT_DIM) = Bk;

        int_Ak(&states[p.STATE_DIM*i],Ak.data(),params);
        for( j = 0; j < i; j++){
            state_sens.block(p.STATE_DIM*i,p.INPUT_DIM*j,p.STATE_DIM,p.INPUT_DIM) = Ak*state_sens.block(p.STATE_DIM*(i-1),p.INPUT_DIM*j,p.STATE_DIM,p.INPUT_DIM);
        }
    }
}
