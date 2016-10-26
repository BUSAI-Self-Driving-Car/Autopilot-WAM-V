#include "QuadraticProgram.h"
#include "qpip_sub.h"
#include "blasmap.h"

QuadraticProgram::QuadraticProgram(const Parameters& params)
    : p(params)
{ }


void QuadraticProgram::operator ()(double* H, double *f, double *L, double *k,double *lb, double *ub, double*x)
{
    double A[4*p.HORIZON*4*p.HORIZON];
    double b[4*p.HORIZON];
    varint li[4*p.HORIZON];
    varint ui[4*p.HORIZON];
    double y[4*p.HORIZON];
    double z[4*p.HORIZON];
    double t[4*p.HORIZON];
    double g[4*p.HORIZON];
    double error;
    int i;

    for( i = 0; i < 4*p.HORIZON; i++){
        li[i] = i;
        ui[i] = i;
    }

    error = qpip_sub(p.HORIZON*4,0,p.HORIZON*4*2,p.HORIZON*4,p.HORIZON*4,H,f,L,k,A,b,lb,li,ub,ui,x,y,z,t,g,0,0,1);
}
