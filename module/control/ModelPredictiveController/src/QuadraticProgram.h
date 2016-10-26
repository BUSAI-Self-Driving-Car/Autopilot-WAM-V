#ifndef QUADRATICPROGRAM_H
#define QUADRATICPROGRAM_H

#include "Parameters.hpp"

class QuadraticProgram
{
public:
    QuadraticProgram(const Parameters& params);
    void operator() (double* H, double *f, double *L, double *k,double *lb, double *ub, double*x);

private:
    const Parameters& p;
};

#endif // QUADRATICPROGRAM_H
