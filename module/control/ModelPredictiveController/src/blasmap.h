/* This file maps the blas calls for linux systems */

#ifndef QPC_BLASMAP_H_
#define QPC_BLASMAP_H_

/* Map BLAS calls for *nix based machines */
#ifdef OS_LIN
#define dsytrf dsytrf_
#define dsytrs dsytrs_
#define dpotrf dpotrf_
#define dpotrs dpotrs_
#define dcopy  dcopy_
#define ddot   ddot_
#define dtrsv  dtrsv_
#define dgemv  dgemv_
#define dscal  dscal_
#define drotg  drotg_
#define drot   drot_
#define daxpy  daxpy_
#define dsymv  dsymv_
#define dsyrk  dsyrk_
#define idamax idamax_
#endif

/*Prototypes*/
void   dpotrf (char *, long int *, double *, long int *, long int *);
void   dpotrs (char *, long int *, long int *, double *, long int *, double *, long int *, long int *);
void   dsytrf (char *, long int *, double *, long int *, long int *, double *, long int *, long int *);
void   dsytrs (char *, long int *, long int *, double *, long int *, long int *, double *, long int *, long int *);
void   dcopy  (long int *, double *, long int *, double *, long int *);
double ddot   (long int *, double *, long int *, double *, long int *);
void   dtrsv  (char *, char *, char *, long int *, double *, long int *, double *, long int *);
void   dgemv  (char *, long int *, long int *, double *, double *, long int *, double *, long int *, double *, double *, long int *);
void   dscal  (long int *, double *, double *, long int *);
void   drotg  (double *, double *, double *, double *);
void   drot   (long int *, double *, long int *, double *, long int *, double *, double *);
void   daxpy  (long int *, double *, double *, long int *, double *, long int *);
void   dsymv  (char *, long int *, double *, double *, long int *, double *, long int *, double *, double *, long int *);
void   dsytrf (char *, long int *, double *, long int *, long int *, double *, long int *, long int *);
void   dsyrk  (char *, char *, long int *, long int *, double *, double *, long int *, double *, double *, long int *);
long int    idamax (long int *, double *, long int *);


#endif
