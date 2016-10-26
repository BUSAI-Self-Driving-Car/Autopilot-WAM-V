#ifndef GRADIENTJACOBIAN_H
#define GRADIENTJACOBIAN_H

#include "Parameters.hpp"

class GradientJacobian
{
public:
    GradientJacobian(const Parameters& param);
    void operator() (double *x_pointer, double *inputs_pointer, double *e_pointer, double *grad_pointer, double *J_pointer, double *params);

private:
    const Parameters& p;

    void jac_state(double *x0, double *out, double *params);
    void wrt_inputs(double *x0, double *inputs, double *out, double *params);
    void int_Bk(double *x0, double *inputs, double *out, double *params);
    void int_Ak(double *x0, double *out, double *params);
    void state_sens(double *inputs, double *states, double *out, double *params);
};

#endif // GRADIENTJACOBIAN_H
