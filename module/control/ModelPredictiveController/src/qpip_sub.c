/* This function uses an interior-point method to compute
   the solution of the following Quadratic Program (QP).
 
			min  0.5x'Hx + f'x  s.t.  l(i) <= x(i) for i \in L
			 x                        x(i) <= u(i) for i \in U
 
   In the above, H should be a positive semi-definite and symmetric
   matrix of dimension (n*n). The vector f is assumed to be of
   dimension n. The vector l denotes element-wise lower bounds on x,
   where the lower bounds are present for integers in the set L.
   Similarly, the vector u denotes element-wise upper bounds on x,
   where the upper bounds are present for all integers in U.
   If i \in L and U then the program will test if l(i) <= u(i).
   If not it will exit early returning the appropriate error number
   (see below).
 
   A call to this function will look as follows:
 
   err = qps(n,ncl,ncu,H,f,l,li,u,ui,x,display);
 
   where the variables are assumed to be as follows:
 
   n:         A positive integer value which describes the
			  dimension of x (and consequently H and f).
		  The program will not change n on exit.
 
   ncl:       A non-negative integer value which describes the
			  number of element-wise lower bounds.
 
   ncu:       A non-negative integer value which describes the
			  number of element-wise upper bounds.
 
   H:         Pointer to a double array of length n*n.
 
			  A positive semi-definite and symmetric matrix of
			  dimension (n*n). Note that only the upper triangular
		  components of H are used, hence symmetry is forced.
		  Positive semi-definiteness of H is NOT checked. The
		  program will not change H on exit.
 
   f:         Pointer to a double array of length n.
 
			  A vector of dimension n. The program will not change
			  f on exit.
 
   l:         Pointer to a double array of length ncl.
 
			  A vector of dimension ncl, where each element l(i) is a
			  lower bound on x(li(i)). The program will not change l on
		  exit.
 
   li:        Pointer to an integer array of dimension ncl.
 
			  A vector of dimension ncl whose elements describe which
		  element in x to bound (starting from 0).
 
   u:         Pointer to a double array of length n.
 
			  A vector of dimension n, where each element u(i) is an
			  upper bound on x(i). The program will not change u on
		  exit.
 
   ui:        Pointer to an integer array of dimension ncu.
 
			  A vector of dimension ncu whose elements describe which
		  element in x to bound (starting from 0).
 
   x:         Pointer to a double array of length n.
 
			  A vector of dimension n. This is the vector of primal
			  variables and will hold the most current iterate of the
		  program on exit. If the program runs to completion
		  without error, this is deemed as a solution to the
		  simple QP given above.
 
   display:   An integer value. If display is less than 1, then no
			  information (error messages included) will be displayed.
		  If display equals 1, then error messages only will
		  be displayed. If display is greater than 1, then both
		  error messages and iteration information will be displayed.
 
   err:	      An integer value.
 
			  On exit, err will be an integer with the following
		  meaning:
 
		  err = 5:  There is not enough memory to solve this problem.
 
		  err = 0:  The program ran to completion without error.
 
		  err = 1:  The program exceeded the maximum iteration
					count (as determined at the beginning of this
			file with #define itlim ?).
 
		  err = 2:  The Hessian matrix used internally is not
					positive definite within working precision.
			The program will not terminate early in this
			case, but rather try to run to completion which
			might mean that the answer is satisfactory.
 
		  err = 3:  The constraints are inconsistent, i.e.
					l(i) > u(i) for some i. The program will terminate
						early in this case.
 
		  err = 4:  Either li or ui has an entry greater than n-1 or
					less than zero. */

/*-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------
 
  +----------------------------------------------+
  | Written by Adrian Wills,                     |
  |            School of Elec. Eng. & Comp. Sci. |
  |            University of Newcastle,          |
  |            Callaghan, NSW, 2308, AUSTRALIA   |
  |                                              |
  | Last Revised  25 May 2007.                   |
  |                                              |
  | Copyright (C) Adrian Wills.                  |
  +----------------------------------------------+
 
The current version of this software is free of charge and 
openly distributed, BUT PLEASE NOTE:

This software must be referenced when used in a published work.

This software may not be re-distributed as a part of a commercial product. 
If you distribute it in a non-commercial products, please contact me first, 
to make sure you ship the most recent version.

This software is distributed in the hope that it will be useful, 
but WITHOUT ANY WARRANTY; without even the implied warranty of 
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 
IF IT FAILS TO WORK, IT'S YOUR LOSS AND YOUR PROBLEM.

-------------------------------------------------------------------------*/
#ifdef INT_INT
#define varint int
#endif

#ifdef INT_LONG_INT
#define varint long int
#endif

#ifdef INT_LONG_LONG_INT
#define varint long long int
#endif

#include <stdlib.h>
#include <math.h>
#include "qpip_sub.h"
#include "blasmap.h"

/* Various constant values I use throughout*/
#define eps 2.2204e-16
#define seps 1e-07
#define mu_n_eps 1e-04
#define itlim 40
#define shift_offset 10.0

/* Constants used in Gondzio corrector */
#define gamma_f 0.99
#define gamma_a 100 /* 1/(1-gamma_f) */
#define MaxNumGond 20
#define AcceptTol 0.005
#define beta_min 0.1
#define beta_max 10
#define StepFactor1 1.08
#define StepFactor0 0.08
/*-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*/
double maxabs(double, varint, double *, varint);
/*-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*/
/*                           MAIN QPIP ROUTINE                             */
/*-------------------------------------------------------------------------*/
varint qpip_sub(varint n,
varint ne,
varint nc,
varint ncl,
varint ncu,
double *  H,
double *  f,
double *  L,
double *  k,
double *  A,
double *  b,
double *  l,
varint *  li,
double *  u,
varint *  ui,
double *  x,
double *  y,
double *  z,
double *  t,
double *  g,
double mu_s,
varint display,
varint reduce)
{
  /*-------------------------------------------------------------------------*/
  /* Define variables */
  // double *y; /* Lagrangian multipliers for equality constraints*/
	double *s; /* Slack variables for general inequality constraints */
  // double *z; /* Lagrangian multipliers for general inequality constraints*/
  // double *g; /* Lagrangian multipliers for upper bounds */
	double *w; /* Slack variables for  upper bounds */
  // double *t; /* Lagrangian multipliers for lower bounds */
	double *v; /* Slack variables for lower bounds */
	
	double *dx; /* Primal variables direction */
	double *dy; /* Lagrangian multipliers for equality constraints*/
	double *ds; /* Slack variables for general inequality constraints */
	double *dz; /* Lagrangian multipliers for general inequality constraints*/
	double *dg; /* Lagrangian multipliers for upper bounds */
	double *dw; /* Slack variables for  upper bounds */
	double *dt; /* Lagrangian multipliers for lower bounds */
	double *dv; /* Slack variables for lower bounds */
	
	double *r1; /* Residual of Lagrangian equation */
	double *r2; /* Residual of equality constraints */
	double *r3; /* Residual of general inequality constraints */
	double *r4; /* Residual of general inequality complementarity */
	double *r5; /* Residual of upper bounds equation */
	double *r6; /* Residual of upper bounds complementarity */
	double *r7; /* Residual of lower bounds equation */
	double *r8; /* Residual of lower bounds complementarity */
	double *rhs; /* Combined residual for computing search direction */
	
	double *C; /* Cholesky factor */
	
	double mu; /* Complementarity gap parameter (or barrier weighting parameter) */
	double am; /* Step length variable */
	unsigned int cnt = 0; /* Iteration count variable */
	double data_infn; /* Infinity norm of problem data */
	double res_infn; /* Relative residual variable */
	double sfmu; /* Scaling factor times mu */
	varint i, j, m, err, indx, nec, idx1, idx2;
	double tmp, tmp1, shift, mu_sd, mu0, pres_infn, dres_infn, sdata_infn, phi, phi_min;
	double gap, dres_infn_min, pres_infn_min, *work, opt_lwork, mu_norm, sf, am_predictor, sf1;
	double pres_infn_old, dres_infn_old;
	varint *ipiv, info, lwork;
	
  /* Variables for Gondzio style correctors */
	double rmax, rmin, ae, amt;
	varint NumGond, StopCorrections;
	double *dxc; /* Primal variables direction */
	double *dyc; /* Lagrangian multipliers for equality constraints */
	double *dsc; /* Slack variables for general inequality constraints */
	double *dzc; /* Lagrangian multipliers for general inequality constraints*/
	double *dgc; /* Lagrangian multipliers for upper bounds */
	double *dwc; /* Slack variables for  upper bounds */
	double *dtc; /* Lagrangian multipliers for lower bounds */
	double *dvc; /* Slack variables for lower bounds */
	
  /* Variables for Mehotra's final step length calc */
	varint ampi, amdi, pord;
	double a, mufull;
	double pval, dval;
	double *p, *d, *dp, *dd;
	
  /* Variable to indicate that both upper and lower bounds are
	 present on that element */
	varint *cb1, *cb2;
	
  /* Variable we use to store all the variables */
	double *ptr;
    
  /* Variables (or constants) used in calls to fortran blas and lapack */
    double one  =  1.0;
    double mone = -1.0;
    double zero =  0.0;
    varint ione    =  1;
    varint izero   =  0;
    double scal;
    varint ti;
  /*-------------------------------------------------------------------------*/
	
	
  /*-------------------------------------------------------------------------*/
  /* Run some checks on the input variables */
  /* Check that elements of li and ui are sensible */
	for(i=0;i<ncl;i++)
	{
		if((li[i] < 0) || (li[i] >= n))
		{
			return 4;
		}
	}
	for(i=0;i<ncu;i++)
	{
		if((ui[i] < 0) || (ui[i] >= n))
		{
			if(display >= 1)
			{
				//mexPrintf("\n\nERROR:  The %i'th element of ui is out of range.\n\n",i);
			}
			return 4;
		}
	}
	
  /* Check that constraints make sense and record equality constraints */
	cb1 = (varint *)malloc(n*sizeof(varint));
	cb2 = (varint *)malloc(n*sizeof(varint));
	for(i=0;i++;i<n){cb1[i]=-1;cb2[i]=-1;}
	for(i=0;i<ncu;i++){cb1[ui[i]]=i;}
	for(i=0;i<ncl;i++){cb2[li[i]]=i;}
	for(i=0;i++;i<n){
		if((cb1[i]>=0)&&(cb2[i]>=0)){
			if(l[cb1[i]] > u[cb2[i]]){
				if(display >= 1){
					//mexPrintf("\n\nERROR:  The constraints are inconsistent.\n\n");
				}
				return 3;
			}
		}
	}
	free(cb1);
	free(cb2);
  /*-------------------------------------------------------------------------*/
	
  /*-------------------------------------------------------------------------*/
  /* Allocate memory - I do this (mainly) in one chunck to make sure the program 
     will be able to run...standard stuff */
	nec=n+ne+nc;
	ptr = (double *)malloc((4*ne+13*nc+4*n+12*ncu+12*ncl+nec*nec)*sizeof(double));
	if(ptr == NULL){return 5;}
	indx = 0;
  /*y = &ptr[indx]; indx = indx + ne;*/
	s = &ptr[indx]; indx = indx + nc;
  /*z = &ptr[indx]; indx = indx + nc;*/
  /*g = &ptr[indx]; indx = indx + ncu;*/
	w = &ptr[indx]; indx = indx + ncu;
  /*t = &ptr[indx]; indx = indx + ncl;*/
	v = &ptr[indx]; indx = indx + ncl;
	dx = &ptr[indx]; indx = indx + n;
	dy = &ptr[indx]; indx = indx + ne;
	ds = &ptr[indx]; indx = indx + nc;
	dz = &ptr[indx]; indx = indx + nc;
	dg = &ptr[indx]; indx = indx + ncu;
	dw = &ptr[indx]; indx = indx + ncu;
	dt = &ptr[indx]; indx = indx + ncl;
	dv = &ptr[indx]; indx = indx + ncl;
	dxc = &ptr[indx]; indx = indx + n;
	dyc = &ptr[indx]; indx = indx + ne;
	dsc = &ptr[indx]; indx = indx + nc;
	dzc = &ptr[indx]; indx = indx + nc;
	dgc = &ptr[indx]; indx = indx + ncu;
	dwc = &ptr[indx]; indx = indx + ncu;
	dtc = &ptr[indx]; indx = indx + ncl;
	dvc = &ptr[indx]; indx = indx + ncl;
	r1 = &ptr[indx]; indx = indx + n;
	r2 = &ptr[indx]; indx = indx + ne;
	r3 = &ptr[indx]; indx = indx + nc;
	r4 = &ptr[indx]; indx = indx + nc;
	r5 = &ptr[indx]; indx = indx + ncu;
	r6 = &ptr[indx]; indx = indx + ncu;
	r7 = &ptr[indx]; indx = indx + ncl;
	r8 = &ptr[indx]; indx = indx + ncl;
	rhs = &ptr[indx]; indx = indx + n + ne + nc;
	C = &ptr[indx]; indx = indx + nec*nec;
	p = &ptr[indx]; indx = indx + nc + ncu + ncl;
	d = &ptr[indx]; indx = indx + nc + ncu + ncl;
	dp = &ptr[indx]; indx = indx + nc + ncu + ncl;
	dd = &ptr[indx]; indx = indx + nc + ncu + ncl;
	
  /* Setup dsytrf */
	ipiv = (varint *)malloc(nec*sizeof(varint));
	lwork = -1;
	dsytrf("L", &nec, C, &nec, ipiv, &opt_lwork, &lwork, &info);
	lwork = (int)opt_lwork;
	work  = (double *)malloc(lwork*sizeof(double));
  /*-------------------------------------------------------------------------*/
	
  /*-------------------------------------------------------------------------*/
  /* Main routine starts here, where we initialise the variables */
  /* Scale H and f if mu_s is too large */
	if(mu_s>1.0){
        scal=1.0/mu_s;
        ti=n*n;
        dscal(&n,&scal,f,&ione);
		dscal(&ti,&scal,H,&ione);
		mu_sd=1.0/mu_s; mu_s=1.0;
	}
	else{mu_sd=1.0;}
	
  /* Calculate the maximum absolute value entry in H L k f l and u*/
	data_infn=maxabs(0.0,n*n,H,1);
    data_infn=maxabs(data_infn,n,f,1);
	if(nc>0){
        data_infn=maxabs(data_infn,nc*n,L,1);
        data_infn=maxabs(data_infn,nc,k,1);
	}
	if(ncu>0){data_infn=maxabs(data_infn,ncu,u,1);}
	if(ncl>0){data_infn=maxabs(data_infn,ncl,l,1);}
	sdata_infn=sqrt(data_infn);
	    
  /* Initialise the variables */
	/* If ONLY simple bounds then do quick initialisation */
	if((nc<=0) && (ne<=0))
	{
	/* Temporarily set x and dx to vectors of 1e6 and -1e6 resp. */
		tmp = 10.0;
		dcopy(&n,&tmp,&izero,x,&ione);
		tmp = -10.0;
		dcopy(&n,&tmp,&izero,dx,&ione);
	    /* Set the components of x to the upper bounds in u */
		for(i=0; i<ncu; i++){x[ui[i]]=u[i];}
	    /* Set the components of dx to the lower bounds in l */
		for(i=0; i<ncl; i++){dx[li[i]]=l[i];}
	    /* Set x to (u+l)/2 */
		for(i=0;i<n;i++){x[i]=(x[i]+dx[i])/2;}
	    /* Set g to u-x */
		for(i=0; i<ncu; i++){w[i]=u[i]-x[ui[i]];}
	    /* Set t to x-l */
		for(i=0; i<ncl; i++){v[i]=x[li[i]]-l[i];}
	    /* r1 = H*x + f*/
		dcopy(&n,f,&ione,r1,&ione);
		dgemv("N",&n,&n,&one,H,&n,x,&ione,&one,r1,&ione);
	    /* Set g to (1e3-min(min(r1),0))*ones(n,1) */
		for(i=0;i<ncu;i++){if(r1[ui[i]]<0.0){g[i]=1e3-r1[ui[i]];}else{g[i]=1e3;}}
	    /* Set t to r1+g */
		dcopy(&n,r1,&ione,dx,&ione);
		for(i=0; i<ncu; i++){dx[ui[i]]+=g[i];}
		for(i=0; i<ncl; i++){if(dx[li[i]]<0.0){t[i]=1e3;}else{t[i]=dx[li[i]];}}
	}
	else /* Otherwise do full initialisation */
	{
		tmp=0.0; dcopy(&n,&tmp,&izero,x,&ione);
		if(ne>0){dcopy(&ne,&tmp,&izero,y,&ione);}
		if(nc>0){dcopy(&nc,&sdata_infn,&izero,s,&ione);   dcopy(&nc,&sdata_infn,&izero,z,&ione);}
		if(ncu>0){dcopy(&ncu,&sdata_infn,&izero,g,&ione); dcopy(&ncu,&sdata_infn,&izero,w,&ione);}
		if(ncl>0){dcopy(&ncl,&sdata_infn,&izero,t,&ione); dcopy(&ncl,&sdata_infn,&izero,v,&ione);}
		
        /* Calculate residuals */
		mu=0.0;
		dcopy(&n,f,&ione,r1,&ione);
		if(ne>0){dcopy(&ne,b,&ione,r2,&ione); dscal(&ne,&mone,r2,&ione);}
		if(nc>0){
			dgemv("T",&nc,&n,&one,L,&nc,z,&ione,&one,r1,&ione);
			dcopy(&nc,s,&ione,r3,&ione);
			daxpy(&nc,&mone,k,&ione,r3,&ione);
			dgemv("N",&nc,&n,&one,L,&nc,x,&ione,&one,r3,&ione);
			mu+=ddot(&nc,s,&ione,z,&ione);
		}
		for(i=0;i<ncu;i++){r1[ui[i]]+=g[i]; r5[i]=x[ui[i]]+w[i]-u[i];} mu+=ddot(&ncu,g,&ione,w,&ione);
		for(i=0;i<ncl;i++){r1[li[i]]-=t[i]; r7[i]=v[i]-x[li[i]]+l[i];} mu+=ddot(&ncl,t,&ione,v,&ione);
		mu=mu/(ncl+ncu+nc);
		
        /* Copy H, A and L into C */
		tmp=0.0; for(i=0;i<nec;i++){ti=nec-i;dcopy(&ti,&tmp,&izero,&C[(nec+1)*i],&ione);}
		for(i=0;i<n;i++){ti=n-i;dcopy(&ti,&H[(n+1)*i],&ione,&C[(nec+1)*i],&ione);}
		if(ne>0){for(i=0;i<ne;i++){dcopy(&n,&A[i],&ne,&C[n+i],&nec);}}
		if(nc>0){for(i=0;i<n;i++){dcopy(&nc,&L[nc*i],&ione,&C[n+ne+nec*i],&ione);}}
		
        /* compute r4, r6 and r8 */
		for(i=0;i<nc;i++){r4[i]=s[i]*z[i]-mu;}
		for(i=0;i<ncu;i++){r6[i]=g[i]*w[i]-mu;}
		for(i=0;i<ncl;i++){r8[i]=t[i]*v[i]-mu;}
		
        /* Add diagonal terms to C and get rhs ready */
		dcopy(&n,r1,&ione,rhs,&ione);
		if(ne>0){dcopy(&ne,r2,&ione,&rhs[n],&ione);}
		if(nc>0){
			if(reduce){
	            /* Compute D = S^{-1}Z and Compute Dr3 - S^{-1}r4 and form L^TD^{1/2} */
				for(i=0;i<nc;i++){
					d[i]=z[i]/s[i];
					p[i]=d[i]*r3[i]-r4[i]/s[i];
					scal=sqrt(d[i]);dscal(&n,&scal,&C[n+ne+i],&nec);
				}
	            /* Compute L^TDL and add to C */
				dsyrk("L","T",&n,&nc,&one,&C[n+ne],&nec,&one,C,&nec);
				
	            /* Update rhs */
				dgemv("T",&nc,&n,&one,L,&nc,p,&ione,&one,rhs,&ione);
			}
			else{
				idx1=(n+ne)*(nec+1);
				for(i=0;i<nc;i++){
					C[idx1+i*(nec+1)]=-s[i]/z[i];
					rhs[n+ne+i]=r3[i]-r4[i]/z[i];
				}
			}
		}
		if(ncu>0){
			for(i=0;i<ncu;i++){
				C[(nec+1)*ui[i]]+=g[i]/w[i];
				rhs[ui[i]]+=(g[i]*r5[i]-r6[i])/w[i];
			}
		}
		if(ncl>0){
			for(i=0;i<ncl;i++){
				C[(nec+1)*li[i]]+=t[i]/v[i];
				rhs[li[i]]-=(t[i]*r7[i]-r8[i])/v[i];
			}
		}
		
        /* Factor C matrix and solve for rhs */
		if(reduce){i=n+ne;}else{i=nec;}
		dsytrf("L", &i, C, &nec, ipiv, work, &lwork, &info);
		if(info!=0)
		{
			if(display>0){
				//mexPrintf("There was an error factoring matrix: info = %i\n\n",info);
			}
			err=11;
			goto exit_stub;
		}
		idx1=1;
		dsytrs("L",&i,&idx1,C,&nec,ipiv,rhs,&nec,&info);
		if(info!=0)
		{
			if(display>0)
			{
				//mexPrintf("There was an error solving for search direction: info = %i\n\n",info);
			}
			err=12;
			goto exit_stub;
		}
		
        /* Get search directions */
		dcopy(&n,rhs,&ione,dx,&ione);
		if(ne>0){dcopy(&ne,&rhs[n],&ione,dy,&ione);}
		if(nc>0){
			if(reduce){
				dgemv("N",&nc,&n,&one,L,&nc,dx,&ione,&zero,dz,&ione);
				for(i=0;i<nc;i++){dz[i]=d[i]*dz[i]-p[i]; ds[i]=(r4[i]-s[i]*dz[i])/z[i];}
			}
			else {
				dcopy(&nc,&rhs[n+ne],&ione,dz,&ione);
				for(i=0;i<nc;i++){ds[i]=(r4[i]-s[i]*dz[i])/z[i];}
			}
		}
		for(i=0;i<ncu;i++){dw[i]=r5[i]-dx[ui[i]]; dg[i]=(r6[i]-g[i]*dw[i])/w[i];}
		for(i=0;i<ncl;i++){dv[i]=r7[i]+dx[li[i]]; dt[i]=(r8[i]-t[i]*dv[i])/v[i];}
		
        /* Take full step in search direction */
		daxpy(&n,&mone,dx,&ione,x,&ione);
		if(ne>0){daxpy(&ne,&mone,dy,&ione,y,&ione);}
		if(nc>0){daxpy(&nc,&mone,dz,&ione,z,&ione); daxpy(&nc,&mone,ds,&ione,s,&ione);}
		if(ncu>0){daxpy(&ncu,&mone,dg,&ione,g,&ione); daxpy(&ncu,&mone,dw,&ione,w,&ione);}
		if(ncl>0){daxpy(&ncl,&mone,dt,&ione,t,&ione); daxpy(&ncl,&mone,dv,&ione,v,&ione);}
		
        /* Calculate shift and shift variables */
		shift=0.0;
		for(i=0;i<nc;i++){if(s[i]<shift){shift=s[i];} if(z[i]<shift){shift=z[i];}}
		for(i=0;i<ncu;i++){if(g[i]<shift){shift=g[i];} if(w[i]<shift){shift=w[i];}}
		for(i=0;i<ncl;i++){if(t[i]<shift){shift=t[i];} if(v[i]<shift){shift=v[i];}}
		shift = shift_offset - 2*shift;
		if(nc>0){daxpy(&nc,&one,&shift,&izero,z,&ione); daxpy(&nc,&one,&shift,&izero,s,&ione);}
		if(ncu>0){daxpy(&ncu,&one,&shift,&izero,g,&ione); daxpy(&ncu,&one,&shift,&izero,w,&ione);}
		if(ncl>0){daxpy(&ncl,&one,&shift,&izero,t,&ione); daxpy(&ncl,&one,&shift,&izero,v,&ione);}
	}
	
	
	
  /* Calculate residuals */
	dcopy(&n,f,&ione,r1,&ione);
	dgemv("N",&n,&n,&one,H,&n,x,&ione,&one,r1,&ione);
	dcopy(&n,r1,&ione,dx,&ione); /* save H*x+f into dx for later */
	if(ne>0){
		dgemv("T",&ne,&n,&one,A,&ne,y,&ione,&one,r1,&ione);
		dcopy(&ne,b,&ione,r2,&ione);
		dgemv("N",&ne,&n,&one,A,&ne,x,&ione,&mone,r2,&ione);
	}
	if(nc>0){
		dgemv("T",&nc,&n,&one,L,&nc,z,&ione,&one,r1,&ione);
		dcopy(&nc,s,&ione,r3,&ione);
		daxpy(&nc,&mone,k,&ione,r3,&ione);
		dgemv("N",&nc,&n,&one,L,&nc,x,&ione,&one,r3,&ione);
	}
	for(i=0;i<ncu;i++){r1[ui[i]]+=g[i]; r5[i]=x[ui[i]]+w[i]-u[i];}
	for(i=0;i<ncl;i++){r1[li[i]]-=t[i]; r7[i]=v[i]-x[li[i]]+l[i];}
	
  /* Update the vital variables mu and res_infn */
	gap=ddot(&n,dx,&ione,x,&ione); mu=0.0; mu_norm=0.0;
	dres_infn=maxabs(0.0,n,r1,1);
	pres_infn = 0.0;
	if(ne>0){
		gap += ddot(&ne,b,&ione,y,&ione);
		pres_infn=maxabs(pres_infn,ne,r2,1);
	}
	if(nc>0) {
		gap += ddot(&nc,k,&ione,z,&ione);
		pres_infn=maxabs(pres_infn,nc,r3,1);
		for(i=0;i<nc;i++){tmp=s[i]*z[i]; if(fabs(tmp-mu_s)>mu_norm){mu_norm=fabs(tmp-mu_s);} mu+=tmp;}
	}
	if(ncu>0) {
		gap += ddot(&ncu,u,&ione,g,&ione);
		pres_infn=maxabs(pres_infn,ncu,r5,1);
		for(i=0;i<ncu;i++){tmp=g[i]*w[i]; if(fabs(tmp-mu_s)>mu_norm){mu_norm=fabs(tmp-mu_s);} mu+=tmp;}
	}
	if(ncl>0) {
		gap -= ddot(&ncl,l,&ione,t,&ione);
        pres_infn=maxabs(pres_infn,ncl,r7,1);
		for(i=0;i<ncl;i++){tmp=t[i]*v[i]; if(fabs(tmp-mu_s)>mu_norm){mu_norm=fabs(tmp-mu_s);} mu+=tmp;}
	}
	if(pres_infn>dres_infn){res_infn=pres_infn;}else{res_infn=dres_infn;}
	gap = fabs(gap);
	phi = (res_infn + gap)/data_infn;
	res_infn = res_infn/data_infn;
	if(nc+ncu+ncl>0){mu = mu/(ncu+ncl+nc);} else{mu=0.0;}
	mu0 = mu;
	phi_min = phi;
	pres_infn_min = pres_infn;
	dres_infn_min = dres_infn;
	
	if(display>0){
		//mexPrintf("--------------------------------------------------------------------------------\n");
		//mexPrintf("%5s%15s%15s%15s%15s%15s\n","It#","Residual","Comp. Gap","Cent. Meas.","#Correctors","Step Length");
		//mexPrintf("--------------------------------------------------------------------------------\n");
		//mexPrintf("%5i%15.5e%15.5e%15.5e%15s%15s\n",cnt+1,res_infn,mu/mu_sd,mu_norm,"n/a","n/a");
	}
  /*-------------------------------------------------------------------------*/
	
  /*-------------------------------------------------------------------------*/
  /* Main while loop */
	while((cnt<itlim) && ((res_infn>seps) || (fabs(mu-mu_s)>seps) || (mu_norm>mu_n_eps))) {
	/* Update the iteration count */
		cnt++;
		
	/*-------------------------------------------------------------------------*/
	/* Compute predictor step */
	/* Copy H, A and L into C */
		tmp=0.0; for(i=0;i<nec;i++){ti=nec-i;dcopy(&ti,&tmp,&izero,&C[(nec+1)*i],&ione);}
		for(i=0;i<n;i++){ti=n-i;dcopy(&ti,&H[(n+1)*i],&ione,&C[(nec+1)*i],&ione);}
		if(ne>0){for(i=0;i<ne;i++){dcopy(&n,&A[i],&ne,&C[n+i],&nec);}}
		if(nc>0){for(i=0;i<n;i++){dcopy(&nc,&L[nc*i],&ione,&C[n+ne+nec*i],&ione);}}
		
	/* compute r4, r6 and r8 */
		for(i=0;i<nc;i++){r4[i]=s[i]*z[i]-mu_s;}
		for(i=0;i<ncu;i++){r6[i]=g[i]*w[i]-mu_s;}
		for(i=0;i<ncl;i++){r8[i]=t[i]*v[i]-mu_s;}
		
	/* Add diagonal terms to C and get rhs ready */
		dcopy(&n,r1,&ione,rhs,&ione);
		if(ne>0){dcopy(&ne,r2,&ione,&rhs[n],&ione);}
		if(nc>0){
			if(reduce){
	/* Compute D = S^{-1}Z and Compute Dr3 - S^{-1}r4 and form L^TD^{1/2} */
				for(i=0;i<nc;i++){
					d[i]=z[i]/s[i];
					p[i]=d[i]*r3[i]-r4[i]/s[i];
					scal=sqrt(d[i]); dscal(&n,&scal,&C[n+ne+i],&nec);
				}
	/* Compute L^TDL and add to C */
				dsyrk("L","T",&n,&nc,&one,&C[n+ne],&nec,&one,C,&nec);
				
	/* Update rhs */
				dgemv("T",&nc,&n,&one,L,&nc,p,&ione,&one,rhs,&ione);
			}
			else{
				idx1=(n+ne)*(nec+1);
				for(i=0;i<nc;i++){
					C[idx1+i*(nec+1)]=-s[i]/z[i];
					rhs[n+ne+i]=r3[i]-r4[i]/z[i];
				}
			}
		}
		if(ncu>0){
			for(i=0;i<ncu;i++){
				C[(nec+1)*ui[i]]+=g[i]/w[i];
				rhs[ui[i]]+=(g[i]*r5[i]-r6[i])/w[i];
			}
		}
		if(ncl>0){
			for(i=0;i<ncl;i++){
				C[(nec+1)*li[i]]+=t[i]/v[i];
				rhs[li[i]]-=(t[i]*r7[i]-r8[i])/v[i];
			}
		}
		
	/* Factor C matrix and solve for rhs */
		if(reduce){i=n+ne;}else{i=nec;}
		dsytrf("L", &i, C, &nec, ipiv, work, &lwork, &info);
		if(info!=0)
		{
			if(display>0)
			{
				//mexPrintf("There was an error factoring matrix: info = %i\n\n",info);
			}
			err=11;
			goto exit_stub;
		}
		idx1=1; 
		dsytrs("L",&i,&idx1,C,&nec,ipiv,rhs,&nec,&info);
        if(info!=0)
		{
			if(display>0)
			{
				//mexPrintf("There was an error solving for search direction: info = %i\n\n",info);
			}
			err=12;
			goto exit_stub;
		}
	/* Get search directions */
		dcopy(&n,rhs,&ione,dx,&ione);
		if(ne>0){dcopy(&ne,&rhs[n],&ione,dy,&ione);}
		if(nc>0){
			if(reduce){
				dgemv("N",&nc,&n,&one,L,&nc,dx,&ione,&zero,dz,&ione);
				for(i=0;i<nc;i++){dz[i]=d[i]*dz[i]-p[i]; ds[i]=(r4[i]-s[i]*dz[i])/z[i];}
			}
			else {
				dcopy(&nc,&rhs[n+ne],&ione,dz,&ione);
				for(i=0;i<nc;i++){ds[i]=(r4[i]-s[i]*dz[i])/z[i];}
			}
		}
		for(i=0;i<ncu;i++){dw[i]=r5[i]-dx[ui[i]]; dg[i]=(r6[i]-g[i]*dw[i])/w[i];}
		for(i=0;i<ncl;i++){dv[i]=r7[i]+dx[li[i]]; dt[i]=(r8[i]-t[i]*dv[i])/v[i];}
	/*-------------------------------------------------------------------------*/
		
		
	/*-------------------------------------------------------------------------*/
	/* Compute maximum distance to boundary and scaling factor */
	/* Compute am = 1/max([max(dz./z) max(ds./s) max(dg./g) max(dt./t) 1]); */
		tmp = 1.0;
		for(i=0;i<nc;i++){
			tmp1=ds[i]/s[i];if(tmp1>tmp){tmp=tmp1;}
			tmp1=dz[i]/z[i];if(tmp1>tmp){tmp=tmp1;}
		}
		for(i=0;i<ncu;i++){
			tmp1=dg[i]/g[i];if(tmp1>tmp){tmp=tmp1;}
			tmp1=dw[i]/w[i];if(tmp1>tmp){tmp=tmp1;}
		}
		for(i=0;i<ncl;i++){
			tmp1=dt[i]/t[i];if(tmp1>tmp){tmp=tmp1;}
			tmp1=dv[i]/v[i];if(tmp1>tmp){tmp=tmp1;}
		}
		am = 1.0/tmp; am_predictor=am;
	/*-------------------------------------------------------------------------*/
		
	/*-------------------------------------------------------------------------*/
	/* sf = (((g - am*dg)'*(s - am*ds) + (t - am*dt)'*(z - am*dz))/(2*n*mu))^3 */
		tmp = 0;
		for(i=0;i<nc;i++){tmp += (s[i]-am*ds[i])*(z[i]-am*dz[i]);}
		for(i=0;i<ncu;i++){tmp += (g[i]-am*dg[i])*(w[i]-am*dw[i]);}
		for(i=0;i<ncl;i++){tmp += (t[i]-am*dt[i])*(v[i]-am*dv[i]);}
		tmp = fabs(tmp - (ncu+ncl+nc)*mu_s)/((ncu+ncl+nc)*mu);
		sf = tmp*tmp*tmp;
		sfmu = mu*sf;
	/*-------------------------------------------------------------------------*/
		
	/*-------------------------------------------------------------------------*/
	/* Calculate affine scaling direction */
		for(i=0;i<nc;i++){r4[i] = s[i]*z[i] + ds[i]*dz[i] - mu_s - sfmu;}
		for(i=0;i<ncu;i++){r6[i] = g[i]*w[i] + dg[i]*dw[i] - mu_s - sfmu;}
		for(i=0;i<ncl;i++){r8[i] = t[i]*v[i] + dt[i]*dv[i] - mu_s - sfmu;}
		
	/* Get rhs ready */
		dcopy(&n,r1,&ione,rhs,&ione);
		if(ne>0){dcopy(&ne,r2,&ione,&rhs[n],&ione);}
		if(nc>0){
			if(reduce){
				for(i=0;i<nc;i++){p[i]=d[i]*r3[i]-r4[i]/s[i];}
				dgemv("T",&nc,&n,&one,L,&nc,p,&ione,&one,rhs,&ione);
			}
			else{
				for(i=0;i<nc;i++){rhs[n+ne+i]=r3[i]-r4[i]/z[i];}
			}
		}
		if(ncu>0){for(i=0;i<ncu;i++){rhs[ui[i]]+=(g[i]*r5[i]-r6[i])/w[i];}}
		if(ncl>0){for(i=0;i<ncl;i++){rhs[li[i]]-=(t[i]*r7[i]-r8[i])/v[i];}}
		
		
	/* Solve for rhs */
		if(reduce){i=n+ne;}else{i=nec;}
		idx1=1; 
		dsytrs("L",&i,&idx1,C,&nec,ipiv,rhs,&nec,&info);
		if(info!=0)
		{
			if(display>0)
			{
				//mexPrintf("There was an error solving for search direction: info = %i\n\n",info);
			}
			err=12;
			goto exit_stub;
		}
		
	/* Get search directions */
		dcopy(&n,rhs,&ione,dxc,&ione);
		if(ne>0){dcopy(&ne,&rhs[n],&ione,dyc,&ione);}
		if(nc>0){
			if(reduce){
				dgemv("N",&nc,&n,&one,L,&nc,dxc,&ione,&zero,dzc,&ione);
				for(i=0;i<nc;i++){dzc[i]=d[i]*dzc[i]-p[i]; dsc[i]=(r4[i]-s[i]*dzc[i])/z[i];}
			}
			else {
				dcopy(&nc,&rhs[n+ne],&ione,dzc,&ione);
				for(i=0;i<nc;i++){dsc[i]=(r4[i]-s[i]*dzc[i])/z[i];}
			}
		}
		for(i=0;i<ncu;i++){dwc[i]=r5[i]-dxc[ui[i]]; dgc[i]=(r6[i]-g[i]*dwc[i])/w[i];}
		for(i=0;i<ncl;i++){dvc[i]=r7[i]+dxc[li[i]]; dtc[i]=(r8[i]-t[i]*dvc[i])/v[i];}
	/*-------------------------------------------------------------------------*/
		
	/*-------------------------------------------------------------------------*/
	/* Compute maximum distance to boundary and scaling factor */
		tmp = 1.0;
		for(i=0;i<nc;i++){
			tmp1=dsc[i]/s[i];if(tmp1>tmp){tmp=tmp1;}
			tmp1=dzc[i]/z[i];if(tmp1>tmp){tmp=tmp1;}
		}
		for(i=0;i<ncu;i++){
			tmp1=dgc[i]/g[i];if(tmp1>tmp){tmp=tmp1;}
			tmp1=dwc[i]/w[i];if(tmp1>tmp){tmp=tmp1;}
		}
		for(i=0;i<ncl;i++){
			tmp1=dtc[i]/t[i];if(tmp1>tmp){tmp=tmp1;}
			tmp1=dvc[i]/v[i];if(tmp1>tmp){tmp=tmp1;}
		}
		am = 1.0/tmp;
	/*-------------------------------------------------------------------------*/
		
	/*-------------------------------------------------------------------------*/
	/* sf = (((g - am*dg)'*(s - am*ds) + (t - am*dt)'*(z - am*dz))/(2*n*mu))^3 */
		tmp = 0;
		for(i=0;i<nc;i++){tmp += (s[i]-am*dsc[i])*(z[i]-am*dzc[i]);}
		for(i=0;i<ncu;i++){tmp += (g[i]-am*dgc[i])*(w[i]-am*dwc[i]);}
		for(i=0;i<ncl;i++){tmp += (t[i]-am*dtc[i])*(v[i]-am*dvc[i]);}
		tmp = fabs(tmp - (ncu+ncl+nc)*mu_s)/((ncu+ncl+nc)*mu);
		sf1 = tmp*tmp*tmp;
	/*-------------------------------------------------------------------------*/
		
	/*-------------------------------------------------------------------------*/
	/* Determine if we should take the corrector step or not! */
		if(sf1<sf){
			dcopy(&n,dxc,&ione,dx,&ione);
			if(ne>0){dcopy(&ne,dyc,&ione,dy,&ione);}
			if(nc>0){dcopy(&nc,dsc,&ione,ds,&ione); dcopy(&nc,dzc,&ione,dz,&ione);}
			if(ncu>0){dcopy(&ncu,dgc,&ione,dg,&ione); dcopy(&ncu,dwc,&ione,dw,&ione);}
			if(ncl>0){dcopy(&ncl,dtc,&ione,dt,&ione); dcopy(&ncl,dvc,&ione,dv,&ione);}
		} else {am=am_predictor;}
	/*-------------------------------------------------------------------------*/
		
	/*-------------------------------------------------------------------------*/
	/* Gondzio style multiple correctors */
	/* Note that first three residuals are set to zero */
		rmin = sfmu*beta_min;
		rmax = sfmu*beta_max;
		
		StopCorrections = 0;
		NumGond = 0;
		
		while((NumGond < MaxNumGond) && (am < 1.0) && (StopCorrections == 0)){
	  /* Compute full step */
			dcopy(&n,x,&ione,dxc,&ione);
			if(ne>0){dcopy(&ne,y,&ione,dyc,&ione);}
			if(nc>0){dcopy(&nc,s,&ione,dsc,&ione); dcopy(&nc,z,&ione,dzc,&ione);}
			if(ncu>0){dcopy(&ncu,g,&ione,dgc,&ione); dcopy(&ncu,w,&ione,dwc,&ione);}
			if(ncl>0){dcopy(&ncl,t,&ione,dtc,&ione); dcopy(&ncl,v,&ione,dvc,&ione);}
			amt = StepFactor1*am + StepFactor0;
			if(amt > 1){amt = 1.0;}
			scal=-amt;
            daxpy(&n,&scal,dx,&ione,dxc,&ione);
			if(ne>0){daxpy(&ne,&scal,dy,&ione,dyc,&ione);}
			if(nc>0){daxpy(&nc,&scal,ds,&ione,dsc,&ione); daxpy(&nc,&scal,dz,&ione,dzc,&ione);}
			if(ncu>0){daxpy(&ncu,&scal,dg,&ione,dgc,&ione); daxpy(&ncu,&scal,dw,&ione,dwc,&ione);}
			if(ncl>0){daxpy(&ncl,&scal,dt,&ione,dtc,&ione); daxpy(&ncl,&scal,dv,&ione,dvc,&ione);}
			
	  /* Set r4, r6 and r8 */
			for(i=0;i<nc;i++){r4[i]=dsc[i]*dzc[i]-mu_s;}
			for(i=0;i<ncu;i++){r6[i]=dgc[i]*dwc[i]-mu_s;}
			for(i=0;i<ncl;i++){r8[i]=dtc[i]*dvc[i]-mu_s;}
			
	  /* Compute projection for r4 and r5 */
			for(i=0;i<nc;i++){
				if(r4[i] < rmin){r4[i]=rmin-r4[i];}
				else if(r4[i]>rmax){r4[i]=rmax-r4[i];}
				else {r4[i]=0.0;}
				if(r4[i]<-rmax){r4[i]=-rmax;}
			}
			
			for(i=0;i<ncu;i++){
				if(r6[i] < rmin){r6[i]=rmin-r6[i];}
				else if(r6[i]>rmax){r6[i]=rmax-r6[i];}
				else {r6[i]=0.0;}
				if(r6[i]<-rmax){r6[i]=-rmax;}
			}
			for(i=0;i<ncl;i++){
				if(r8[i] < rmin){r8[i]=rmin-r8[i];}
				else if(r8[i]>rmax){r8[i]=rmax-r8[i];}
				else {r8[i]=0.0;}
				if(r8[i]<-rmax){r8[i]=-rmax;}
			}
			
	  /* Get rhs ready */
			tmp = 0.0; dcopy(&nec,&tmp,&izero,rhs,&ione);
			if(nc>0){
				if(reduce){
					for(i=0;i<nc;i++){p[i]=-r4[i]/s[i];}
					dgemv("T",&nc,&n,&one,L,&nc,p,&ione,&one,rhs,&ione);
				}
				else{
					for(i=0;i<nc;i++){rhs[n+ne+i]=-r4[i]/z[i];}
				}
			}
			if(ncu>0){for(i=0;i<ncu;i++){rhs[ui[i]]-=r6[i]/w[i];}}
			if(ncl>0){for(i=0;i<ncl;i++){rhs[li[i]]+=r8[i]/v[i];}}
			
			
	  /* Solve for rhs */
			if(reduce){i=n+ne;}else{i=nec;}
			idx1=1; 
			dsytrs("L",&i,&idx1,C,&nec,ipiv,rhs,&nec,&info);
			if(info!=0)
			{
				if(display>0)
				{
					//mexPrintf("There was an error solving for search direction: info = %i\n\n",info);
				}
				err=12;
				goto exit_stub;
			}
			
	  /* Get search directions */
			dcopy(&n,rhs,&ione,dxc,&ione);
			if(ne>0){dcopy(&ne,&rhs[n],&ione,dyc,&ione);}
			if(nc>0){
				if(reduce){
					dgemv("N",&nc,&n,&one,L,&nc,dxc,&ione,&zero,dzc,&ione);
					for(i=0;i<nc;i++){dzc[i]=d[i]*dzc[i]-p[i]; dsc[i]=(r4[i]-s[i]*dzc[i])/z[i];}
				}
				else {
					dcopy(&nc,&rhs[n+ne],&ione,dzc,&ione);
					for(i=0;i<nc;i++){dsc[i]=(r4[i]-s[i]*dzc[i])/z[i];}
				}
			}
			if(ncu>0){for(i=0;i<ncu;i++){rhs[ui[i]]-=r6[i]/w[i];}}
			if(ncl>0){for(i=0;i<ncl;i++){rhs[li[i]]+=r8[i]/v[i];}}
			for(i=0;i<ncu;i++){dwc[i]=-dxc[ui[i]]; dgc[i]=(r6[i]-g[i]*dwc[i])/w[i];}
			for(i=0;i<ncl;i++){dvc[i]=dxc[li[i]]; dtc[i]=(r8[i]-t[i]*dvc[i])/v[i];}
	  /*-------------------------------------------------------------------------*/
			
	  /* Compute maximum distance to boundary and scaling factor */
	  /* Compute am = 1/max([max(dz./z) max(ds./s) max(dg./g) max(dt./t) 1]); */
			tmp = 1.0;
			for(i=0;i<nc;i++){
				tmp1=(ds[i]-dsc[i])/s[i];if(tmp1>tmp){tmp=tmp1;}
				tmp1=(dz[i]-dzc[i])/z[i];if(tmp1>tmp){tmp=tmp1;}
			}
			for(i=0;i<ncu;i++){
				tmp1=(dg[i]-dgc[i])/g[i];if(tmp1>tmp){tmp=tmp1;}
				tmp1=(dw[i]-dwc[i])/w[i];if(tmp1>tmp){tmp=tmp1;}
			}
			for(i=0;i<ncl;i++){
				tmp1=(dt[i]-dtc[i])/t[i];if(tmp1>tmp){tmp=tmp1;}
				tmp1=(dv[i]-dvc[i])/v[i];if(tmp1>tmp){tmp=tmp1;}
			}
			ae=1.0/tmp; if(ae>1.0){ae=1.0;}
			
			if(ae == 1.0) {
				daxpy(&n,&mone,dxc,&ione,dx,&ione);
				if(ne>0){daxpy(&ne,&mone,dyc,&ione,dy,&ione);}
				if(nc>0){daxpy(&nc,&mone,dsc,&ione,ds,&ione); daxpy(&nc,&mone,dzc,&ione,dz,&ione);}
				if(ncu>0){daxpy(&ncu,&mone,dgc,&ione,dg,&ione); daxpy(&ncu,&mone,dwc,&ione,dw,&ione);}
				if(ncl>0){daxpy(&ncl,&mone,dtc,&ione,dt,&ione); daxpy(&ncl,&mone,dvc,&ione,dv,&ione);}
				am = ae;
				NumGond++;
				StopCorrections = 1;
			} else if(ae >= (1.0+AcceptTol)*am) {
				daxpy(&n,&mone,dxc,&ione,dx,&ione);
				if(ne>0){daxpy(&ne,&mone,dyc,&ione,dy,&ione);}
				if(nc>0){daxpy(&nc,&mone,dsc,&ione,ds,&ione); daxpy(&nc,&mone,dzc,&ione,dz,&ione);}
				if(ncu>0){daxpy(&ncu,&mone,dgc,&ione,dg,&ione); daxpy(&ncu,&mone,dwc,&ione,dw,&ione);}
				if(ncl>0){daxpy(&ncl,&mone,dtc,&ione,dt,&ione); daxpy(&ncl,&mone,dvc,&ione,dv,&ione);}
				am = ae;
				NumGond++;
				StopCorrections = 0;
			} else {StopCorrections = 1;}
		}
	/*-------------------------------------------------------------------------*/
		
	/*-------------------------------------------------------------------------*/
	/* Compute final step length using Mehotras heuristic */
		if(nc+ncu+ncl>0){
			if(nc>0){
				dcopy(&nc,s,&ione,p,&ione);
				dcopy(&nc,ds,&ione,dp,&ione);
				dcopy(&nc,z,&ione,d,&ione);
				dcopy(&nc,dz,&ione,dd,&ione);
			}
			if(ncu>0){
				dcopy(&ncu,w,&ione,&p[nc],&ione);
				dcopy(&ncu,dw,&ione,&dp[nc],&ione);
				dcopy(&ncu,g,&ione,&d[nc],&ione);
				dcopy(&ncu,dg,&ione,&dd[nc],&ione);
			}
			if(ncl>0){
				dcopy(&ncl,v,&ione,&p[nc+ncu],&ione);
				dcopy(&ncl,dv,&ione,&dp[nc+ncu],&ione);
				dcopy(&ncl,t,&ione,&d[nc+ncu],&ione);
				dcopy(&ncl,dt,&ione,&dd[nc+ncu],&ione);
			}
			pval = dp[0]/p[0]; ampi = 0;
			dval = dd[0]/d[0]; amdi = 0;
			for(i=1;i<(nc+ncu+ncl);i++){
				tmp=dp[i]/p[i];if(tmp>pval){pval=tmp;ampi=i;}
				tmp=dd[i]/d[i];if(tmp>dval){dval=tmp;amdi=i;}
			}
			
			pord=0;am=pval;if(dval>pval){pord=1;am=dval;}
			am=1/am;
			
			a = 1.0;
			if(am >= 1){a=1.0; am = 1.0;}
			else{
				mufull = 0;
				for(i=0;i<(nc+ncu+ncl);i++){
					mufull += (p[i]-am*dp[i])*(d[i]-am*dd[i]);
				}
				mufull = (0.01*mufull)/(nc+ncu+ncl);
				
				if(pord==0){
					a = (p[ampi] - mufull/(d[amdi] - am*dd[amdi]))/dp[ampi];
				}
				else{
					a = (d[amdi] - mufull/(p[ampi] - am*dp[ampi]))/dd[amdi];
				}
			}
	  /* Make sure the step length is sensible */
			if(a > 1.0){a=1.0;}
			if(a < 0.99*am){a=0.99*am;}
	  /* Back off a smidge */
			am = a*0.99999999;
		} else {am=1.0;} /* There are no inequality constraints so take full step*/
	/*-------------------------------------------------------------------------*/
		
		
	/*-------------------------------------------------------------------------*/
	/* Update the iterates */
		scal=-am;
        daxpy(&n,&scal,dx,&ione,x,&ione);
		if(ne>0){daxpy(&ne,&scal,dy,&ione,y,&ione);}
		if(nc>0){daxpy(&nc,&scal,ds,&ione,s,&ione); daxpy(&nc,&scal,dz,&ione,z,&ione);}
		if(ncu>0){daxpy(&ncu,&scal,dg,&ione,g,&ione); daxpy(&ncu,&scal,dw,&ione,w,&ione);}
		if(ncl>0){daxpy(&ncl,&scal,dt,&ione,t,&ione); daxpy(&ncl,&scal,dv,&ione,v,&ione);}
	/*-------------------------------------------------------------------------*/
		
	/*-------------------------------------------------------------------------*/
	/* Store feasibility data */
		pres_infn_old = pres_infn;
		dres_infn_old = dres_infn;
		if(phi<phi_min){phi_min=phi;}
		if(pres_infn<pres_infn_min){pres_infn_min=pres_infn;}
		if(dres_infn<dres_infn_min){dres_infn_min=dres_infn;}
	/*-------------------------------------------------------------------------*/
		
	/* Calculate residuals */
		dcopy(&n,f,&ione,r1,&ione);
		dgemv("N",&n,&n,&one,H,&n,x,&ione,&one,r1,&ione);
		dcopy(&n,r1,&ione,dx,&ione); /* save H*x+f into dx for later */
		if(ne>0){
			dgemv("T",&ne,&n,&one,A,&ne,y,&ione,&one,r1,&ione);
			dcopy(&ne,b,&ione,r2,&ione);
			dgemv("N",&ne,&n,&one,A,&ne,x,&ione,&mone,r2,&ione);
		}
		if(nc>0){
			dgemv("T",&nc,&n,&one,L,&nc,z,&ione,&one,r1,&ione);
			dcopy(&nc,s,&ione,r3,&ione);
			daxpy(&nc,&mone,k,&ione,r3,&ione);
			dgemv("N",&nc,&n,&one,L,&nc,x,&ione,&one,r3,&ione);
		}
		for(i=0;i<ncu;i++){r1[ui[i]]+=g[i]; r5[i]=x[ui[i]]+w[i]-u[i];}
		for(i=0;i<ncl;i++){r1[li[i]]-=t[i]; r7[i]=v[i]-x[li[i]]+l[i];}
		
	/* Update the vital variables mu and res_infn */
		gap=ddot(&n,dx,&ione,x,&ione); mu=0.0; mu_norm=0.0;
		dres_infn=maxabs(0.0,n,r1,1);
        pres_infn = 0.0;
        if(ne>0){
            gap += ddot(&ne,b,&ione,y,&ione);
            pres_infn=maxabs(pres_infn,ne,r2,1);
        }
        if(nc>0) {
            gap += ddot(&nc,k,&ione,z,&ione);
            pres_infn=maxabs(pres_infn,nc,r3,1);
            for(i=0;i<nc;i++){tmp=s[i]*z[i]; if(fabs(tmp-mu_s)>mu_norm){mu_norm=fabs(tmp-mu_s);} mu+=tmp;}
        }
        if(ncu>0) {
            gap += ddot(&ncu,u,&ione,g,&ione);
            pres_infn=maxabs(pres_infn,ncu,r5,1);
            for(i=0;i<ncu;i++){tmp=g[i]*w[i]; if(fabs(tmp-mu_s)>mu_norm){mu_norm=fabs(tmp-mu_s);} mu+=tmp;}
        }
        if(ncl>0) {
            gap -= ddot(&ncl,l,&ione,t,&ione);
            pres_infn=maxabs(pres_infn,ncl,r7,1);
            for(i=0;i<ncl;i++){tmp=t[i]*v[i]; if(fabs(tmp-mu_s)>mu_norm){mu_norm=fabs(tmp-mu_s);} mu+=tmp;}
        }
		if(pres_infn>dres_infn){res_infn=pres_infn;}else{res_infn=dres_infn;}
		gap = fabs(gap);
		phi = (res_infn + gap)/data_infn;
		res_infn = res_infn/data_infn;
		pres_infn = pres_infn/data_infn;
		dres_infn = dres_infn/data_infn;
		mu = mu/(ncu+ncl+nc);
		if(display>0){
			//mexPrintf("%5i%15.5e%15.5e%15.5e%15i%15.5e\n",cnt+1,res_infn,mu/mu_sd,mu_norm,NumGond,am);
		}
		
	/*-------------------------------------------------------------------------*/
	/* Run various checks for convergence */
		if((pres_infn>seps) && ((pres_infn_old-pres_infn)/pres_infn_min<seps)){
			if(display>0){/*mexPrintf("\n\nWarning: Primal feasibility gap is not decreasing, exiting early.\n\t Primal problem may be infeasible.\n\n");*/}
			err=2; goto exit_stub;
		}
		if((dres_infn>seps) && ((dres_infn_old-dres_infn)/dres_infn_min<seps)){
			if(display>0){/*mexPrintf("\n\nWarning: Dual feasibility gap is not decreasing, exiting early.\n\t Primal problem may be unbounded below.\n\n");*/}
			err=3; goto exit_stub;
		}
		/*
         if((phi>seps) && (phi>1e8*phi_min)){
			if(display>0){mexPrintf("\n\nWarning: Feasibility gap is not decreasing, exiting early.\n\t Primal problem may be infeasible.\n\n");}
			err=4; goto exit_stub;
		} 
         */
	/*-------------------------------------------------------------------------*/
		
	/*-------------------------------------------------------------------------*/
	/*                              END MAIN LOOP                              */
	/*-------------------------------------------------------------------------*/
	}
	if(display>0){
//		mexPrintf("--------------------------------------------------------------------------------\n");
	}
	if(cnt>=itlim) 
	{
		if(display>0){/*mexPrintf("\n\nWARNING:  Hit maximum iteration count.\n\n");*/}
		err=-1;
	}
	else
	{
		err=0;
	}
	
	exit_stub:
		free(ipiv);
		free(work);
		free(ptr);
		return err;
}


/*-------------------------------------------------------------------------*/
double maxabs(double maxabsin, varint n, double *x, varint incx)
{
    double tmp;
    double mar=maxabsin;
    varint i;
    for(i=0; i<n; i++){
        tmp=fabs(x[i*incx]);
        if (tmp>mar){mar=tmp;}
    }
    
    return mar;
}
/*-------------------------------------------------------------------------*/

#undef eps
#undef seps
#undef itlim
#undef gamma_f
#undef gamma_a
#undef MaxNumGond
#undef AcceptTol
#undef beta_min
#undef beta_max
#undef StepFactor1
#undef StepFactor0
