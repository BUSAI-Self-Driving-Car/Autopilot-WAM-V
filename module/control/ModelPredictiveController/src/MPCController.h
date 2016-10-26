#ifndef MPCCONTROLLER_H
#define MPCCONTROLLER_H

#include "Parameters.hpp"

class MPCController
{

public:

    MPCController();

    MPCController(Parameters parameters);

    void setParameters(Parameters parameters);

    void operator() (double* x0, double* xref, double* u_in_p, double* uprev, double* u_out_p, double* param_est);

private:
    Parameters p;

    void setupConstraints(double* A_p, double* b_p, double* uprev_p);

    void statesSim(double* x0, double* inputs, double* out, double* params);

    void dstate(double* x0, double* inputs, double* out, double* params);

    void eCost(double* x_pointer, double* xref_pointer, double* inputs_pointer, double* uprev_pointer, double* cost, double* e_pointer);

    void merit(double *u, double *x0, double *xref, double *uprev, double *cost, double *x, double *e, double *params);

    void linesearch(double *originalcost, double *u_in_p, double *d_p, double *D, double *x0, double *xref, double *uprev, double *u_out_p, double *x_out_p, double *e_out_p, double *cost, double *alpha, double *iters, double *params);

    int convergence(double *unew_pointer, double *u_pointer, double *xnew_pointer, double *x_pointer);
};

#endif // MPCCONTROLLER_H
