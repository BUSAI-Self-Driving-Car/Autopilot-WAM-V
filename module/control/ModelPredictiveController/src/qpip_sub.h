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
#ifdef __cplusplus
extern "C" {
#endif

#ifndef QPIP_H_CBLAS
#define QPIP_H_CBLAS

#ifdef INT_INT
#define varint int
#endif

#ifdef INT_LONG_INT
#define varint long int
#endif

#ifdef INT_LONG_LONG_INT
#define varint long long int
#endif

/* The main function */
varint qpip_sub( 
varint       n,
varint       ne,
varint       nc,
varint       ncl,
varint       ncu,
double *  H,
double *  f,
double *  L,
double *  k,
double *  A,
double *  b,
double *  l,
varint    *  li,
double *  u,
varint    *  ui,
double *  x,
double *  y,
double *  z,
double *  t,
double *  g,
double    mu_s,
varint       display,
varint       reduce
);

#endif


#ifdef __cplusplus
}
#endif