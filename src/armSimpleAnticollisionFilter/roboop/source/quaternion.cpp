/*
Copyright (C) 2002-2004  Etienne Lachance

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

email: etienne.lachance@polymtl.ca or richard.gourdeau@polymtl.ca

-------------------------------------------------------------------------------
Revision_history:

2004/01/19: Etienne Lachance
    -Removed function Exp and Ln.
    -Added function power and Log.
    -Fixed bugs in Slerp, Slerp_prime, Squad and Squad_prime.

2003/05/23: Etienne Lachance
    -Added the following member function -=, +=, *=, /=, Exp, Ln, dot, d_dt, E
    -Added functions Integ_Trap_quat, Slerp, Slerp_prime, Squad, Squad_prime,

2004/05/14: Etienne Lachance
    -Replaced vec_x_prod by CrossProduct.

2004/05/21: Etienne Lachance
   -Added comments that can be used with Doxygen.

2004/07/01: Etienne Lachance
   -Replaced vec_dot_prod by DotProdut of Newmat library.

2004/07/01: Ethan Tira-Thompson
    -Added support for newmat's use_namespace #define, using ROBOOP namespace
    -Fixed problem in constructor using float as Real type

2005/11/06: Etienne Lachance
    - No need to provide a copy constructor and the assignment operator 
      (operator=) for Quaternion class. Instead we use the one provide by the
      compiler.

2005/11/13: Etienne Lachance
    - operator* and operator/ are now non-member functions when one of the
      operand is a real. With these modifications we support q2 = c * q1 and
      q2 = q1 * c
-------------------------------------------------------------------------------
*/


/*!
  @file quaternion.cpp
  @brief Quaternion functions.
*/

//! @brief RCS/CVS version.
static const char rcsid[] = "$Id: quaternion.cpp,v 1.1 2007/07/24 16:03:10 amaldo Exp $";

#include "quaternion.h"

#ifdef use_namespace
namespace ROBOOP {
  using namespace NEWMAT;
#endif


Quaternion::Quaternion()
//! @brief Constructor.
{
   s_ = 1.0;
   v_ = ColumnVector(3);
   v_ = 0.0;
}

Quaternion::Quaternion(const Real angle, const ColumnVector & axis)
//! @brief Constructor.
{
   if(axis.Nrows() != 3)
   {
      cerr << "Quaternion::Quaternion, size of axis != 3" << endl;
      exit(1);
   }

   // make sure axis is a unit vector
   Real norm_axis = sqrt(DotProduct(axis, axis));

   if(norm_axis != 1)
   {
      cerr << "Quaternion::Quaternion(angle, axis), axis is not unit" << endl;
      cerr << "Make the axis unit." << endl;
      v_ = sin(angle/2) * axis/norm_axis;
   }
   else
      v_ = sin(angle/2) * axis;

   s_ = cos(angle/2);
}

Quaternion::Quaternion(const Real s_in, const Real v1, const Real v2,
                       const Real v3)
//!  @brief Constructor.
{
   s_ = s_in;
   v_ = ColumnVector(3);
   v_(1) = v1;
   v_(2) = v2;
   v_(3) = v3;
}

Quaternion::Quaternion(const Matrix & R)
/*!
  @brief Constructor.

  Cite_: Dam.
  The unit quaternion obtained from a matrix (see Quaternion::R())
  \f[
  R(s,v) =
   \left[
    \begin{array}{ccc}
      s^2+v_1^2-v_2^2-v_3^2 & 2v_1v_2+2sv_3 & 2v_1v_3-2sv_2 \\
      2v_1v_2-2sv_3 & s^2-v_1^2+v_2^2-v_3^2 & 2v_2v_3+2sv_1 \\
      2v_1v_3+2sv_2 &2v_2v_3-2sv_1 & s^2-v_1^2-v_2^2+v_3^2  
    \end{array}
   \right]
  \f]
  
  First we find \f$s\f$:
  \f[
   R_{11} + R_{22} + R_{33} + R_{44} = 4s^2
  \f]
  Now the other values are:
  \f[
    s = \pm \frac{1}{2}\sqrt{R_{11} + R_{22} + R_{33} + R_{44}}
  \f]
  \f[
   v_1 = \frac{R_{32}-R_{23}}{4s}
  \f]
  \f[
   v_2 = \frac{R_{13}-R_{31}}{4s}
  \f]
  \f[
   v_3 = \frac{R_{21}-R_{12}}{4s}
  \f]

  The sign of \f$s\f$ cannot be determined. Depending on the choice of the sign 
  for s the sign of \f$v\f$ change as well. Thus the quaternions \f$q\f$ and 
  \f$-q\f$ represent the same rotation, but the interpolation curve changed with
  the choice of the sign. 
  A positive sign has been chosen.
*/
{
   if( (R.Nrows() == 3) && (R.Ncols() == 3) ||
         (R.Nrows() == 4) && (R.Ncols() == 4) )
   {
      Real tmp = fabs(R(1,1) + R(2,2) + R(3,3) + 1);
      s_ = 0.5*sqrt(tmp);
      if(v_.Nrows() != 3)
         v_ = ColumnVector(3);

      if(s_ > EPSILON)
      {
         v_(1) = (R(3,2)-R(2,3))/(4*s_);
         v_(2) = (R(1,3)-R(3,1))/(4*s_);
         v_(3) = (R(2,1)-R(1,2))/(4*s_);
      }
      else
      {
         // |w| <= 1/2
         static int s_iNext[3] = { 2, 3, 1 };
         int i = 1;
         if ( R(2,2) > R(1,1) )
            i = 2;
         if ( R(3,3) > R(2,2) )
            i = 3;
         int j = s_iNext[i-1];
         int k = s_iNext[j-1];

         Real fRoot = sqrt(R(i,i)-R(j,j)-R(k,k) + 1.0);

         Real *tmp[3] = { &v_(1), &v_(2), &v_(3) };
         *tmp[i-1] = 0.5*fRoot;
         fRoot = 0.5/fRoot;
         s_ = (R(k,j)-R(j,k))*fRoot;
         *tmp[j-1] = (R(j,i)+R(i,j))*fRoot;
         *tmp[k-1] = (R(k,i)+R(i,k))*fRoot;
      }

   }
   else
      cerr << "Quaternion::Quaternion: matrix input is not 3x3 or 4x4" << endl;
}

Quaternion Quaternion::operator+(const Quaternion & rhs)const
/*!
  @brief Overload + operator.

  The quaternion addition is 
  \f[
  q_1 + q_2 = [s_1, v_1] + [s_2, v_2] = [s_1+s_2, v_1+v_2]
  \f]

  The result is not necessarily a unit quaternion even if \f$q_1\f$ and
  \f$q_2\f$ are unit quaternions.
*/
{
   Quaternion q;
   q.s_ = s_ + rhs.s_;
   q.v_ = v_ + rhs.v_;

   return q;
}

Quaternion Quaternion::operator-(const Quaternion & rhs)const
/*!
  @brief Overload - operator.

  The quaternion soustraction is 
  \f[
  q_1 - q_2 = [s_1, v_1] - [s_2, v_2] = [s_1-s_2, v_1-v_2]
  \f]

  The result is not necessarily a unit quaternion even if \f$q_1\f$ and
  \f$q_2\f$ are unit quaternions.
*/
{
   Quaternion q;
   q.s_ = s_ - rhs.s_;
   q.v_ = v_ - rhs.v_;

   return q;
}

Quaternion Quaternion::operator*(const Quaternion & rhs)const
/*!
  @brief Overload * operator.

  The multiplication of two quaternions is

  \f[
  q = q_1q_2 = [s_1s_2 - v_1\cdot v_2, v_1 \times v_2 + s_1v_2 + s_2v_1]
  \f]
  where \f$\cdot\f$ and \f$\times\f$ denote the scalar and vector product
  in \f$R^3\f$ respectively.

  If \f$q_1\f$ and \f$q_2\f$ are unit quaternions, then q will also be a 
  unit quaternion.
*/
{
   Quaternion q;
   q.s_ = s_ * rhs.s_ - DotProduct(v_, rhs.v_);
   q.v_ = s_ * rhs.v_ + rhs.s_ * v_ + CrossProduct(v_, rhs.v_);

   return q;
}


Quaternion Quaternion::operator/(const Quaternion & rhs)const
//! @brief Overload / operator.
{
    return *this*rhs.i();
}


void Quaternion::set_v(const ColumnVector & v)
//! @brief Set quaternion vector part.
{
   if(v.Nrows() == 3)
      v_ = v;
   else
       cerr << "Quaternion::set_v: input has a wrong size." << endl;
}

Quaternion Quaternion::conjugate()const
/*!
  @brief Conjugate.

  The conjugate of a quaternion \f$q = [s, v]\f$ is
  \f$q^{*} = [s, -v]\f$
*/
{
   Quaternion q;
   q.s_ = s_;
   q.v_ = -1*v_;

   return q;
}

Real Quaternion::norm()const 
/*!
  @brief Return the quaternion norm.

  The norm of quaternion is defined by
  \f[
  N(q) = s^2 + v\cdot v
  \f]
*/
{ 
  return( sqrt(s_*s_ + DotProduct(v_, v_)) );
}

Quaternion & Quaternion::unit()
//! @brief Normalize a quaternion.
{
   Real tmp = norm();
   if(tmp > EPSILON)
   {
      s_ = s_/tmp;
      v_ = v_/tmp;
   }
   return *this;
}

Quaternion Quaternion::i()const 
/*!
  @brief Quaternion inverse.
  \f[
    q^{-1} = \frac{q^{*}}{N(q)}
  \f]
  where \f$q^{*}\f$ and \f$N(q)\f$ are the quaternion
  conjugate and the quaternion norm respectively.
*/
{ 
    return conjugate()/norm();
}

Quaternion Quaternion::exp() const
/*!
  @brief Exponential of a quaternion.

  Let a quaternion of the form \f$q = [0, \theta v]\f$, q is not
  necessarily a unit quaternion. Then the exponential function
  is defined by \f$q = [\cos(\theta),v \sin(\theta)]\f$.
*/
{
   Quaternion q;
   Real theta = sqrt(DotProduct(v_,v_)),
                sin_theta = sin(theta);

   q.s_ = cos(theta);
   if ( fabs(sin_theta) > EPSILON)
      q.v_ = v_*sin_theta/theta;
   else
      q.v_ = v_;

   return q;
}

Quaternion Quaternion::power(const Real t) const
{
   Quaternion q = (Log()*t).exp();

   return q;
}

Quaternion Quaternion::Log()const
/*!
  @brief Logarithm of a unit quaternion.

  The logarithm function of a unit quaternion 
  \f$q = [\cos(\theta), v \sin(\theta)]\f$ is defined as 
  \f$log(q) = [0, v\theta]\f$. The result is not necessary 
  a unit quaternion.
*/
{
   Quaternion q;
   q.s_ = 0;
   Real theta = acos(s_),
                sin_theta = sin(theta);

   if ( fabs(sin_theta) > EPSILON)
      q.v_ = v_/sin_theta*theta;
   else
      q.v_ = v_;

   return q;
}

Quaternion Quaternion::dot(const ColumnVector & w, const short sign)const
/*!
  @brief Quaternion time derivative.

  The quaternion time derivative, quaternion propagation equation, is
  \f[
    \dot{s} = - \frac{1}{2}v^Tw_{_0}
  \f]
  \f[
    \dot{v} = \frac{1}{2}E(s,v)w_{_0}
  \f]
  \f[
    E = sI - S(v)
  \f]
  where \f$w_{_0}\f$ is the angular velocity vector expressed in the base
  frame. If the vector is expressed in the object frame, \f$w_{_b}\f$, the
  time derivative becomes
  \f[
   \dot{s} = - \frac{1}{2}v^Tw_{_b}
  \f]
  \f[
   \dot{v} = \frac{1}{2}E(s,v)w_{_b}
  \f]
  \f[
   E = sI + S(v)
  \f]
*/
{
   Quaternion q;
   Matrix tmp;

   tmp = -0.5*v_.t()*w;
   q.s_ = tmp(1,1);
   q.v_ = 0.5*E(sign)*w;

   return q;
}

ReturnMatrix Quaternion::E(const short sign)const
/*!
  @brief Matrix E.

  See Quaternion::dot for explanation.
*/
{
   Matrix E(3,3), I(3,3);
   I << threebythreeident;

   if(sign == BODY_FRAME)
      E = s_*I + x_prod_matrix(v_);
   else
      E = s_*I - x_prod_matrix(v_);

   E.Release();
   return E;
}

Real Quaternion::dot_prod(const Quaternion & q)const
/*!
  @brief Quaternion dot product.

  The dot product of quaternion is defined by  
  \f[
  q_1\cdot q_2 = s_1s_2 + v_1 \cdot v_2
  \f]
*/
{
   return (s_*q.s_ + v_(1)*q.v_(1) + v_(2)*q.v_(2) + v_(3)*q.v_(3));
}

ReturnMatrix Quaternion::R()const
/*!
  @brief Rotation matrix from a unit quaternion.

  \f$p'=qpq^{-1} = Rp\f$ where \f$p\f$ is a vector, \f$R\f$ a rotation
  matrix and \f$q\f$ q quaternion. The rotation matrix obtained from a 
  quaternion is then
  \f[
  R(s,v) = (s^2 - v^Tv)I + 2vv^T - 2s S(v)
  \f]
  \f[
  R(s,v) =
   \left[
    \begin{array}{ccc}
      s^2+v_1^2-v_2^2-v_3^2 & 2v_1v_2+2sv_3 & 2v_1v_3-2sv_2 \\
      2v_1v_2-2sv_3 & s^2-v_1^2+v_2^2-v_3^2 & 2v_2v_3+2sv_1 \\
      2v_1v_3+2sv_2 &2v_2v_3-2sv_1 & s^2-v_1^2-v_2^2+v_3^2  
    \end{array}
   \right]
  \f]
  where \f$S(\cdot)\f$ is the cross product matrix defined by
  \f[
    S(u) = 
     \left[
      \begin{array}{ccc}
        0 & -u_3 & u_2 \\
	u_3 &0 & -u_1  \\
	-u_2 & u_1 & 0 \\
      \end{array}
     \right]
  \f]
*/
{
   Matrix R(3,3);
   R << threebythreeident;
   R = (1 - 2*DotProduct(v_, v_))*R + 2*v_*v_.t() + 2*s_*x_prod_matrix(v_);

   R.Release();
   return R;
}

ReturnMatrix Quaternion::T()const
/*!
  @brief Transformation matrix from a quaternion.

  See Quaternion::R() for equations.
*/
{
   Matrix T(4,4);
   T << fourbyfourident;
   T.SubMatrix(1,3,1,3) = (1 - 2*DotProduct(v_, v_))*T.SubMatrix(1,3,1,3)
                          + 2*v_*v_.t() + 2*s_*x_prod_matrix(v_);
   T.Release();
   return T;
}

// -------------------------------------------------------------------------------------

Quaternion operator*(const Real c, const Quaternion & q)
/*!
  @brief Overload * operator, multiplication by a scalar.

  \f$q = [s, v]\f$ and let \f$r \in R\f$. Then
  \f$rq = qr = [r, 0][s, v] = [rs, rv]\f$

  The result is not necessarily a unit quaternion even if \f$q\f$ 
  is a unit quaternions.
*/
{
    Quaternion out;
    out.set_s(q.s() * c);
    out.set_v(q.v() * c);
   return out;
}

Quaternion operator*(const Quaternion & q, const Real c)
/*!
  @brief Overload * operator, multiplication by a scalar.
*/
{
   return operator*(c, q);
}


Quaternion operator/(const Real c, const Quaternion & q)
/*!
  @brief Overload / operator, division by a scalar.

  Same explanation as multiplication by scaler.
*/
{
    Quaternion out;
    out.set_s(q.s() / c);
    out.set_v(q.v() / c);
   return out;
}

Quaternion operator/(const Quaternion & q, const Real c)
{
    return operator/(c, q);
}

ReturnMatrix Omega(const Quaternion & q, const Quaternion & q_dot)
/*!
  @brief Return angular velocity from a quaternion and it's time derivative

  See Quaternion::dot for explanation.
*/
{
   Matrix A, B, M;
   UpperTriangularMatrix U;
   ColumnVector w(3);
   A = 0.5*q.E(BASE_FRAME);
   B = q_dot.v();
   if(A.Determinant())
   {
      QRZ(A,U);             //QR decomposition
      QRZ(A,B,M);
      w = U.i()*M;
   }
   else
      w = 0;

   w.Release();
   return w;
}

short Integ_quat(Quaternion & dquat_present, Quaternion & dquat_past,
                 Quaternion & quat, const Real dt)
//! @brief Trapezoidal quaternion integration.
{
   if (dt < 0)
   {
      cerr << "Integ_Trap(quat1, quat2, dt): dt < 0. dt is set to 0." << endl;
      return -1;
   }

   // Quaternion algebraic constraint
   //  Real Klambda = 0.5*(1 - quat.norm_sqr());

   dquat_present.set_s(dquat_present.s() );//+ Klambda*quat.s());
   dquat_present.set_v(dquat_present.v() ); //+ Klambda*quat.v());

   quat.set_s(quat.s() + Integ_Trap_quat_s(dquat_present, dquat_past, dt));
   quat.set_v(quat.v() + Integ_Trap_quat_v(dquat_present, dquat_past, dt));

   dquat_past.set_s(dquat_present.s());
   dquat_past.set_v(dquat_present.v());

   quat.unit();

   return 0;
}

Real Integ_Trap_quat_s(const Quaternion & present, Quaternion & past,
                       const Real dt)
//! @brief Trapezoidal quaternion scalar part integration.
{
   Real integ = 0.5*(present.s()+past.s())*dt;
   past.set_s(present.s());
   return integ;
}

ReturnMatrix Integ_Trap_quat_v(const Quaternion & present, Quaternion & past,
                               const Real dt)
//! @brief Trapezoidal quaternion vector part integration.
{
   ColumnVector integ = 0.5*(present.v()+past.v())*dt;
   past.set_v(present.v());
   integ.Release();
   return integ;
}

Quaternion Slerp(const Quaternion & q0, const Quaternion & q1, const Real t)
/*!
  @brief Spherical Linear Interpolation.

  Cite_:Dam

  The quaternion \f$q(t)\f$ interpolate the quaternions \f$q_0\f$ 
  and \f$q_1\f$ given the parameter \f$t\f$ along the quaternion sphere.
  \f[
   q(t) = c_0(t)q_0 + c_1(t)q_1
  \f]
  where \f$c_0\f$ and \f$c_1\f$ are real functions with \f$0\leq t \leq 1\f$.
  As \f$t\f$ varies between 0 and 1. the values \f$q(t)\f$ varies uniformly 
  along the circular arc from \f$q_0\f$ and \f$q_1\f$. The angle between 
  \f$q(t)\f$ and \f$q_0\f$ is \f$\cos(t\theta)\f$ and the angle between
  \f$q(t)\f$ and \f$q_1\f$ is \f$\cos((1-t)\theta)\f$. Taking the dot product
  of \f$q(t)\f$ and \f$q_0\f$ yields
  \f[
   \cos(t\theta) = c_0(t) + \cos(\theta)c_1(t)
  \f]
  and taking the dot product of \f$q(t)\f$ and \f$q_1\f$ yields
  \f[
   \cos((1-t)\theta) = \cos(\theta)c_0(t) + c_1(t)
  \f]
  These are two equations with \f$c_0\f$ and \f$c_1\f$. The solution is
  \f[
   c_0 = \frac{\sin((1-t)\theta)}{\sin(\theta)}
  \f]
  \f[
   c_1 = \frac{\sin(t\theta)}{sin(\theta)}
  \f]
  The interpolation is then
  \f[
   Slerp(q_0, q_1, t) = \frac{q_0\sin((1-t)\theta)+q_1\sin(t\theta)}{\sin(\theta)}
  \f]
  If \f$q_0\f$ and \f$q_1\f$ are unit quaternions the \f$q(t)\f$ is also a unit
  quaternions. For unit quaternions we have
  \f[
   Slerp(q_0, q_1, t) = q_0(q_0^{-1}q_1)^t
  \f]
  For t = 0 and t = 1 we have
  \f[
   q_0 = Slerp(q_0, q_1, 0)
  \f]
  \f[
   q_1 = Slerp(q_0, q_1, 1)
  \f]
  It is customary to choose the sign G on q1 so that q0.Gq1 >=0 (the angle
  between q0 ang Gq1 is acute). This choice avoids extra spinning caused
  by the interpolated rotations.
*/
{
   if( (t < 0) || (t > 1) )
      cerr << "Slerp(q0, q1, t): t < 0 or t > 1. t is set to 0." << endl;

   if(q0.dot_prod(q1) >= 0)
      return q0*((q0.i()*q1).power(t));
   else
      return  q0*((q0.i()*-1*q1).power(t));
}

Quaternion Slerp_prime(const Quaternion & q0, const Quaternion & q1,
                       const Real t)
/*!
  @brief Spherical Linear Interpolation derivative.

  Cite_: Dam

  The derivative of the function \f$q^t\f$ where \f$q\f$ is a constant
  unit quaternion is
  \f[
   \frac{d}{dt}q^t = q^t log(q)
  \f]
  Using the preceding equation the Slerp derivative is then
  \f[
   Slerp'(q_0, q_1, t) = q_0(q_0^{-1}q_1)^t log(q_0^{-1}q_1)
  \f]

  It is customary to choose the sign G on q1 so that q0.Gq1 >=0 (the angle
  between q0 ang Gq1 is acute). This choice avoids extra spinning caused
  by the interpolated rotations.
  The result is not necessary a unit quaternion.
*/

{
   if( (t < 0) || (t > 1) )
      cerr << "Slerp_prime(q0, q1, t): t < 0 or t > 1. t is set to 0." << endl;

   if(q0.dot_prod(q1) >= 0)
      return Slerp(q0, q1, t)*(q0.i()*q1).Log();
   else
      return Slerp(q0, q1, t)*(q0.i()*-1*q1).Log();
}

Quaternion Squad(const Quaternion & p, const Quaternion & a, const Quaternion & b,
                 const Quaternion & q, const Real t)
/*!
  @brief Spherical Cubic Interpolation.

  Cite_: Dam

  Let four quaternions be \f$q_i\f$ (p), \f$s_i\f$ (a), \f$s_{i+1}\f$ (b) and \f$q_{i+1}\f$ 
  (q) be the ordered vertices of a quadrilateral. Obtain c from \f$q_i\f$ to \f$q_{i+1}\f$ 
  interpolation. Obtain d from \f$s_i\f$ to \f$s_{i+1}\f$ interpolation. Obtain e,
  the final result, from c to d interpolation.
  \f[
   Squad(q_i, s_i, s_{i+1}, q_{i+1}, t) = Slerp(Slerp(q_i,q_{i+1},t),Slerp(s_i,s_{i+1},t), 2t(1-t))
  \f]
  The intermediate quaternion \f$s_i\f$ and \f$s_{i+1}\f$ are given by
  \f[
   s_i = q_i exp\Big ( - \frac{log(q_i^{-1}q_{i+1}) + log(q_i^{-1}q_{i-1})}{4}\Big )
  \f]
*/
{
   if( (t < 0) || (t > 1) )
      cerr << "Squad(p,a,b,q, t): t < 0 or t > 1. t is set to 0." << endl;

   return Slerp(Slerp(p,q,t),Slerp(a,b,t),2*t*(1-t));
}

Quaternion Squad_prime(const Quaternion & p, const Quaternion & a, const Quaternion & b,
                       const Quaternion & q, const Real t)
/*!
  @brief Spherical Cubic Interpolation derivative.

  Cite_: www.magic-software.com


  The derivative of the function \f$q^t\f$ where \f$q\f$ is a constant
  unit quaternion is
  \f[
   \frac{d}{dt}q^t = q^t log(q)
  \f]
  Recalling that \f$log(q) = [0, v\theta]\f$ (see Quaternion::Log()). If the power
  is a function we have
  \f[
   \frac{d}{dt}q^{f(t)} = f'(t)q^{f(t)}log(q)
  \f]
  If \f$q\f$ is a function of time and the power is differentiable function of time
  we have
  \f[
  \frac{d}{dt}(q(t))^{f(t)} = f'(t)(q(t))^{f(t)}log(q) + f(t)(q(t))^{f(t)-1}q'(t)
  \f]
  Using these last three equations Squad derivative can be define. Let 
  \f$U(t)=Slerp(p,q,t)\f$, \f$V(t)=Slerp(q,b,t)\f$, \f$W(t)=U(t)^{-1}V(t)\f$. We then
  have \f$Squad(p,a,b,q,t)=Slerp(U(t),V(t),2t(1-t))=U(t)W(t)^{2t(1-t)}\f$

  \f[
   Squad'(p,a,b,q,t) = \frac{d}{dt}\Big [ UW^{2t(1-t)}\Big ]
  \f]
  \f[
   Squad'(p,a,b,q,t) = U\frac{d}{dt}\Big [ W^{2t(1-t)}\Big ] + U'\Big [W^{2t(1-t)}\Big]
  \f]
  \f[
   Squad'(p,a,b,q,t) = U\Big[(2-4t)W^{2t(1-t)}log(W)+2t(1-t)W^{2t(1-t)-1}W'\Big]
    + U'\Big[W^{2t(1-t)} \Big]
  \f]
  where \f$U'=Ulog(p^{-1}q)\f$, \f$V'=Vlog(a^{-1},b)\f$, \f$W'=U^{-1}V'-U^{-2}U'V\f$
  

  The result is not necessarily a unit quaternion even if all the input quaternions are unit.
*/
{
   if( (t < 0) || (t > 1) )
      cerr << "Squad_prime(p,a,b,q, t): t < 0 or t > 1. t is set to 0." << endl;

   Quaternion q_squad,
   U = Slerp(p, q, t),
       V = Slerp(a, b, t),
           W = U.i()*V,
               U_prime = U*(p.i()*q).Log(),
                         V_prime = V*(a.i()*b).Log(),
                                   W_prime = U.i()*V_prime - U.power(-2)*U_prime*V;

   q_squad = U*( W.power(2*t*(1-t))*W.Log()*(2-4*t) + W.power(2*t*(1-t)-1)*W_prime*2*t*(1-t) )
             + U_prime*( W.power(2*t*(1-t)) );

   return q_squad;
}

#ifdef use_namespace
}
#endif






