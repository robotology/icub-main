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

2003/02/03: Etienne Lachance
   -Added functions x_prod_matrix, vec_dot_prod, Integ_Trap and sign.
   -Working on Runge_Kutta4_etienne.

2004/07/01: Etienne Lachance
   -Removed function vec_x_prod and vec_dot_prod. Now we use CrossProduct
    and DotProduct, from Newmat library.
   -Added doxygen documentation.

2004/07/01: Ethan Tira-Thompson
    -Added support for newmat's use_namespace #define, using ROBOOP namespace

2005/06/10: Carmine Lia
   -Added pinv

2005/06/29 Richard Gourdeau
   -Fixed max() bug for VC++ 6.0
-------------------------------------------------------------------------------
*/

/*!
  @file utils.cpp
  @brief Utility functions.
*/

//! @brief RCS/CVS version.
static const char rcsid[] = "$Id: utils.cpp,v 1.1 2007/07/24 16:03:10 amaldo Exp $";

#include "utils.h"

#ifdef _MSC_VER
#if _MSC_VER  < 1300                       // Microsoft
#ifndef GUARD_minmax_H
#define GUARD_minmax_H
// needed to cope with bug in MS library:
// it fails to define min/max
template <class T> inline T max(const T& a, const T& b)
{
   return (a > b) ? a : b;
}
template <class T> inline T min(const T& a, const T& b)
{
   return (a < b) ? a : b;
}
#endif
#endif
#endif

#ifdef use_namespace
namespace ROBOOP {
  using namespace NEWMAT;
#endif


ReturnMatrix x_prod_matrix(const ColumnVector & x)
//!  @brief Cross product matrix
{
   Matrix S(3,3); S = 0.0;
   S(1,2) = -x(3); S(1,3) =  x(2);
   S(2,1) =  x(3);                 S(2,3) = -x(1);
   S(3,1) = -x(2); S(3,2) =  x(1);

   S.Release(); return (S);
}

ReturnMatrix pinv(const Matrix & M)
/*!
  @brief Matrix pseudo inverse using SVD

  If \f$ A = U^{*}QV \f$ is a singular value decomposition of A,
  then \f$ A^{\dagger} = V^{*}Q^{\dagger}U\f$ where
  \f$ X^{*} \f$ is the conjugate transpose of \f$ X \f$
  and 
  \f$ Q^{\dagger} = 
    \left [
      \begin{array}{cccc}
         1/\sigma_1 & & & \\
                    & 1/\sigma_2& & \\
                    &  & \ddots & \\
                    &  &  & 0
      \end{array}
    \right ]\f$ where the \f$1/\sigma_i \f$ are replaced by 0
     when \f$1/\sigma_i < tol \f$
*/



{
  /* floating point eps */
  const Real eps = numeric_limits<Real>::epsilon(); 
  
  int m = M.nrows();
  int n = M.ncols();
  
  if(n > m)
  {
    Matrix X = pinv(M.t()).t();
    return X;
   }
  
  Matrix U, V;
  
  DiagonalMatrix Q;
  
  Matrix X(n, m);
  
  SVD(M, Q, U, V);
 //cout << "Q\n" << Q << "U\n" << U << "V\n" << V << endl;
  Real tol = max(m,n)*Q(1)*eps;
 // cout << "\ntol\n" << tol << "\neps\n" << eps << endl; 
  // conteggio dei sv != 0
  int r = 0;
  for(int i = 1; i <= Q.size(); i++)
      if(Q(i) > tol)
         r++;
  
  if(r != 0)
  {
     DiagonalMatrix D(r);
     for(int i = 1; i <= r; i++)
  	D(i) = 1/Q(i);
     //cout << V.SubMatrix(1,V.nrows(),1,r) << endl;
     //cout << D << endl;
     //cout << U.SubMatrix(1,U.nrows(),1,r).t() << endl;
     X = V.SubMatrix(1,V.nrows(),1,r)*D*U.SubMatrix(1,U.nrows(),1,r).t();
  
  }
  X.Release();
  return X;
}  

ReturnMatrix Integ_Trap(const ColumnVector & present, ColumnVector & past,
                        const Real dt)
  //!  @brief Trapezoidal integration.
{
   ColumnVector integration(present.Nrows());
   integration = (past+present)*0.5*dt;
   past = present;

   integration.Release();
   return ( integration );
}


void Runge_Kutta4_Real_time(ReturnMatrix (*xdot)(Real time, const Matrix & xin,
						 bool & exit, bool & init),
			    const Matrix & xo, Real to, Real tf, int nsteps)
//!  @brief Fixed step size fourth-order Runge-Kutta integrator.
{
    static Real h, h2, t;
    static Matrix k1, k2, k3, k4, x;
    static bool first_pass = true, init = false, exit = false;

    if(first_pass || init)
    {
	h = (tf-to)/nsteps;
	h2 = h/2.0;
	t = to;
	x = xo;
	first_pass = false;
    }
      k1 = (*xdot)(t, x, exit, init) * h;
      if(exit) return;
      k2 = (*xdot)(t+h2, x+k1*0.5, exit, init)*h;
      if(exit) return;
      k3 = (*xdot)(t+h2, x+k2*0.5, exit, init)*h;
      if(exit) return;
      k4 = (*xdot)(t+h, x+k3, exit, init)*h;
      if(exit) return;
      x = x + (k1*0.5+ k2 + k3 + k4*0.5)*(1.0/3.0);
      t += h;
}

void Runge_Kutta4_Real_time(ReturnMatrix (*xdot)(const Real time, const Matrix & xin),
			    const Matrix & xo, const Real to, const Real tf, const int nsteps)
{
    static Real h, h2, t;
    static Matrix k1, k2, k3, k4, x;
    static bool first_pass = true;

    if(first_pass)
    {
	h = (tf-to)/nsteps;
	h2 = h/2.0;
	t = to;
	x = xo;
	first_pass = false;
    }
      k1 = (*xdot)(t, x) * h;
      t += h2;
      k2 = (*xdot)(t, x+k1*0.5)*h;
      k3 = (*xdot)(t, x+k2*0.5)*h;
      t += h2;
      k4 = (*xdot)(t, x+k3)*h;
      x = x + (k1*0.5+ k2 + k3 + k4*0.5)*(1.0/3.0);
}


void Runge_Kutta4(ReturnMatrix (*xdot)(Real time, const Matrix & xin),
            const Matrix & xo, Real to, Real tf, int nsteps,
            RowVector & tout, Matrix & xout)
//!  @brief Fixed step size fourth-order Runge-Kutta integrator.
{
   Real h, h2, t;
   Matrix k1, k2, k3, k4, x;

   h = (tf-to)/nsteps;
   h2 = h/2.0;
   t = to;
   x = xo;
   xout = Matrix(xo.Nrows(),(nsteps+1)*xo.Ncols());
   xout.SubMatrix(1,xo.Nrows(),1,xo.Ncols()) = x;
   tout = RowVector(nsteps+1);
   tout(1) = to;
   for (int i = 1; i <= nsteps; i++) {
      k1 = (*xdot)(t, x) * h;
      k2 = (*xdot)(t+h2, x+k1*0.5)*h;
      k3 = (*xdot)(t+h2, x+k2*0.5)*h;
      k4 = (*xdot)(t+h, x+k3)*h;
      x = x + (k1*0.5+ k2 + k3 + k4*0.5)*(1.0/3.0);
      t += h;
      tout(i+1) = t;
      xout.SubMatrix(1,xo.Nrows(),i*xo.Ncols()+1,(i+1)*xo.Ncols()) = x;
   }
}

ReturnMatrix rk4(const Matrix & x, const Matrix & dxdt, Real t, Real h,
                 ReturnMatrix (*xdot)(Real time, const Matrix & xin))
/*!
  @brief Compute one Runge-Kutta fourth order step

  adapted from:
  Numerical Recipes in C, The Art of Scientific Computing,
  Press, William H. and Flannery, Brian P. and Teukolsky, Saul A.
  and Vetterling, William T., Cambridge University Press, 1988.
*/
{
   Matrix xt, xout, dxm, dxt;
   Real th, hh, h6;

   hh = h*0.5;
   h6 = h/6.0;
   th = t + hh;
   xt = x + hh*dxdt;
   dxt = (*xdot)(th,xt);
   xt = x + hh*dxt;
   dxm = (*xdot)(th,xt);
   xt = x + h*dxm;
   dxm += dxt;
   dxt = (*xdot)(t+h,xt);
   xout = x+h6*(dxdt+dxt+2.0*dxm);

   xout.Release(); return xout;
}

#define PGROW -0.20
#define PSHRNK -0.25
#define FCOR 0.06666666
#define SAFETY 0.9
#define ERRCON 6.0E-4

void rkqc(Matrix & x, Matrix & dxdt, Real & t, Real htry,
          Real eps, Matrix & xscal, Real & hdid, Real & hnext,
          ReturnMatrix (*xdot)(Real time, const Matrix & xin))
/*!
  @brief Compute one adaptive step based on two rk4.

  adapted from:
  Numerical Recipes in C, The Art of Scientific Computing,
  Press, William H. and Flannery, Brian P. and Teukolsky, Saul A.
  and Vetterling, William T., Cambridge University Press, 1988.
*/
{
   Real tsav, hh, h, temp, errmax;
   Matrix dxsav, xsav, xtemp;

   tsav = t;
   xsav = x;
   dxsav = dxdt;
   h = htry;
   for(;;) {
      hh = 0.5*h;
      xtemp = rk4(xsav,dxsav,tsav,hh,xdot);
      t = tsav+hh;
      dxdt = (*xdot)(t,xtemp);
      x = rk4(xtemp,dxdt,t,hh,xdot);
      t = tsav+h;
      if(t == tsav) {
         cerr << "Step size too small in routine RKQC\n";
         exit(1);
      }
      xtemp = rk4(xsav,dxsav,tsav,h,xdot);
      errmax = 0.0;
      xtemp = x-xtemp;
      for(int i = 1; i <= x.Nrows(); i++) {
         temp = fabs(xtemp(i,1)/xscal(i,1));
         if(errmax < temp) errmax = temp;
      }
      errmax /= eps;
      if(errmax <= 1.0) {
         hdid = h;
         hnext = (errmax > ERRCON ?
                  SAFETY*h*exp(PGROW*log(errmax)) : 4.0*h);
         break;
      }
      h = SAFETY*h*exp(PSHRNK*log(errmax));
   }
   x += xtemp*FCOR;
}

#define MAXSTP 10000
#define TINY 1.0e-30

void odeint(ReturnMatrix (*xdot)(Real time, const Matrix & xin),
            Matrix & xo, Real to, Real tf, Real eps, Real h1, Real hmin,
            int & nok, int & nbad,
            RowVector & tout, Matrix & xout, Real dtsav)
/*!
  @brief Integrate the ordinary differential equation xdot from time to
   to time tf using an adaptive step size strategy 

   adapted from:
   Numerical Recipes in C, The Art of Scientific Computing,
   Press, William H. and Flannery, Brian P. and Teukolsky, Saul A.
   and Vetterling, William T., Cambridge University Press, 1988.
*/
{
   Real tsav, t, hnext, hdid, h;
   RowVector tv(1);

   Matrix xscal, x, dxdt;

   tv = to;
   tout = tv;
   xout = xo;
   xscal = xo;
   t = to;
   h = (tf > to) ? fabs(h1) : -fabs(h1);
   nok = (nbad) = 0;
   x = xo;
   tsav = t;
   for(int nstp = 1; nstp <= MAXSTP; nstp++){
      dxdt = (*xdot)(t,x);
      for(int i = 1; i <= x.Nrows(); i++)
         xscal(i,1) = fabs(x(i,1))+fabs(dxdt(i,1)*h)+TINY;
      if((t+h-tf)*(t+h-to) > 0.0) h = tf-t;
      rkqc(x,dxdt,t,h,eps,xscal,hdid,hnext,xdot);
      if(hdid == h) ++(nok); else ++(nbad);
      if((t-tf)*(tf-to) >= 0.0) {
         xo = x;
         tv = t;
         tout = tout | tv;
         xout = xout | x;
         return;
      }
      if(fabs(t-tsav) > fabs(dtsav)) {
         tv = t;
         tout = tout | tv;
         xout = xout | x;
         tsav = t;
      }
      if(fabs(hnext) <= hmin) {
         cerr << "Step size too small in ODEINT\n";
         cerr << setw(7) << setprecision(3) << (tout & xout).t();
         exit(1);
      }
      h = hnext;
   }
   cerr << "Too many step in routine ODEINT\n";
   exit(1);
}

ReturnMatrix sign(const Matrix & x)
//!  @brief Sign of a matrix.
{
   Matrix out = x;

   for(int i = 1; i <= out.Ncols(); i++) {
      for(int j = 1; j <= out.Nrows(); j++) {
         if(out(j,i) > 0.0) {
            out(j,i) = 1.0;
         } else if(out(j,i) < 0.0) {
            out(j,i) = -1.0;
         }
      }
   }

   out.Release(); return out;
}


short sign(const Real x)
  //!  @brief Sign of real.
{
   short i = 1;
   if(x > 0.0)
      i = 1;
   else
      i = -1;

   return (i);
}

#ifdef use_namespace
}
#endif

