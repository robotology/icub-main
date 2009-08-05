/*
ROBOOP -- A robotics object oriented package in C++
Copyright (C) 1996-2004  Richard Gourdeau

This library is free software; you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation; either version 2.1 of the
License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


Report problems and direct all questions to:

Richard Gourdeau
Professeur Agrege
Departement de genie electrique
Ecole Polytechnique de Montreal
C.P. 6079, Succ. Centre-Ville
Montreal, Quebec, H3C 3A7

email: richard.gourdeau@polymtl.ca

-------------------------------------------------------------------------------
Revision_history:

2004/07/01: Etienne Lachance
   -Added doxygen documentation.

2004/07/01: Ethan Tira-Thompson
    -Added support for newmat's use_namespace #define, using ROBOOP namespace

2004/09/18: Etienne Lachance
   -Added deg2rad rad2deg

2005/06/10: Carmine Lia
   -Added pinv
-------------------------------------------------------------------------------
*/

#ifndef __cplusplus
#error Must use C++ for the type Robot
#endif
#ifndef UTILS_H
#define UTILS_H

/*!
  @file utils.h
  @brief Utility header file.
*/

//! @brief RCS/CVS version.
static const char header_utils_rcsid[] = "$Id: utils.h,v 1.1 2007/07/24 16:03:10 amaldo Exp $";

#ifdef _MSC_VER                         // Microsoft
#pragma warning (disable:4786)  /* Disable decorated name truncation warnings */
#endif
#include <stdio.h>
#include <limits>
#define WANT_STRING                  /* include.h will get string fns */
#define WANT_STREAM                  /* include.h will get stream fns */
#define WANT_FSTREAM                 /* include.h will get fstream fns */
#define WANT_MATH                    /* include.h will get math fns */
                                     /* newmatap.h will get include.h */

#include "newmatap.h"                /* need matrix applications */

#include "newmatio.h"                /* need matrix output routines */

#ifdef use_namespace
namespace ROBOOP {
  using namespace NEWMAT;
#endif

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

#define GRAVITY 9.81

// global variables
extern Real fourbyfourident[];
extern Real threebythreeident[];

// angle conversion
inline double deg2rad(const double angle_deg){ return angle_deg*M_PI/180; }
inline double rad2deg(const double angle_rad){ return angle_rad*180/M_PI; }

// vector operation 

ReturnMatrix x_prod_matrix(const ColumnVector & x);

ReturnMatrix pinv(const Matrix & M);

// numerical analysis tools

ReturnMatrix Integ_Trap(const ColumnVector & present, ColumnVector & past, const Real dt);

void Runge_Kutta4(ReturnMatrix (*xdot)(Real time, const Matrix & xin),
                  const Matrix & xo, Real to, Real tf, int nsteps,
                  RowVector & tout, Matrix & xout);

void Runge_Kutta4_Real_time(ReturnMatrix (*xdot)(Real time, const Matrix & xin),
                            const Matrix & xo, Real to, Real tf, int nsteps);

void Runge_Kutta4_Real_time(ReturnMatrix (*xdot)(Real time, const Matrix & xin,
                            bool & exit, bool & init),
                            const Matrix & xo, Real to, Real tf, int nsteps);

void odeint(ReturnMatrix (*xdot)(Real time, const Matrix & xin),
            Matrix & xo, Real to, Real tf, Real eps, Real h1, Real hmin,
            int & nok, int & nbad,
            RowVector & tout, Matrix & xout, Real dtsav);

ReturnMatrix sign(const Matrix & x);

short sign(const Real x);

const double epsilon = 0.0000001;

inline bool isZero(const double x)
{
    if ( fabs(x) < epsilon)
    {
        return true;
    }
    return false;
}


// translation 
ReturnMatrix trans(const ColumnVector & a);

// rotation matrices 
ReturnMatrix rotx(const Real alpha);
ReturnMatrix roty(const Real beta);
ReturnMatrix rotz(const Real gamma);
ReturnMatrix rotk(const Real theta, const ColumnVector & k);

ReturnMatrix rpy(const ColumnVector & a);
ReturnMatrix eulzxz(const ColumnVector & a);
ReturnMatrix rotd(const Real theta, const ColumnVector & k1, const ColumnVector & k2);


// inverse on rotation matrices 
ReturnMatrix irotk(const Matrix & R);
ReturnMatrix irpy(const Matrix & R);
ReturnMatrix ieulzxz(const Matrix & R);

#ifdef use_namespace
}
#endif

#endif

