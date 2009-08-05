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
   -Member function inertia and acceleration are now part of class Robot_basic.
   -Added torque member funtions to allowed to had load on last link.
   -Changed variable "n" and "f" for "n_nv" and "f_nv" in torque_novelocity.
   -Corrected calculation of wp, vp and n in mRobot::torque and 
    mRobot::torque_novelocity.
   -Removed all member functions related to classes RobotMotor and mRobotMotor.
   -Added motor effect in torque function (see ltorque).
   -Added function call set_qp() and set_qpp in Robot::torque and mRobot::torque.

2003/04/29: Etienne Lachance
   -Corrected vp calculation for prismatic case in mRobot/mRobot_min_para::torque, 
    mRobot_min_para::torque_novelocity.
   -Added functions Robot_basic::acceleration(const ColumnVector & q,const ColumnVector & qp,
                                              const ColumnVector & tau)
2003/11/18: Etienne Lachance
   -Added member function G() (gravity torque) and C() (Coriolis and centrifugal).

2004/05/14: Etienne Lachance
   -Replaced vec_x_prod by CrossProduct.

2004/05/21: Etienne Lachance
   -Added doxygen comments.

2004/07/01: Ethan Tira-Thompson
    -Added support for newmat's use_namespace #define, using ROBOOP namespace

2004/07/13: Ethan Tira-Thompson
    -Re-added the namespace closing brace at the bottom

2006/05/19: Richard Gourdeau
    -Fixed Gear ratio bug for viscous friction (reported by Carmine Lia)
-------------------------------------------------------------------------------
*/
/*!
  @file dynamics.cpp
  @brief Manipulator dynamics functions.
*/

//! @brief RCS/CVS version.
static const char rcsid[] = "$Id: dynamics.cpp,v 1.1 2007/07/24 16:03:09 amaldo Exp $";

#include "robot.h"

#ifdef use_namespace
namespace ROBOOP {
  using namespace NEWMAT;
#endif


ReturnMatrix Robot_basic::inertia(const ColumnVector & q)
//! @brief Inertia of the manipulator.
{
   Matrix M(dof,dof);
   ColumnVector torque(dof);
   set_q(q);
   for(int i = 1; i <= dof; i++) {
      for(int j = 1; j <= dof; j++) {
         torque(j) = (i == j ? 1.0 : 0.0);
      }
      torque = torque_novelocity(torque);
      M.Column(i) = torque;
   }
   M.Release(); return M;
}


ReturnMatrix Robot_basic::acceleration(const ColumnVector & q,
                                       const ColumnVector & qp,
                                       const ColumnVector & tau_cmd)
//! @brief Joints acceleration without contact force.
{
   ColumnVector qpp(dof);
   qpp = 0.0;
   qpp = inertia(q).i()*(tau_cmd-torque(q,qp,qpp));
   qpp.Release(); 
   return qpp;
}

ReturnMatrix Robot_basic::acceleration(const ColumnVector & q, const ColumnVector & qp,
                                       const ColumnVector & tau_cmd, const ColumnVector & Fext,
                                       const ColumnVector & Next)
/*!
  @brief Joints acceleration.

  The robot dynamics is 
  \f[
    B(q)\ddot{q} + C(q,\dot{q})\dot{q} + D\dot{q} + g(q) = \tau - J^T(q)f
  \f]
  then the joint acceleration is
  \f[
   \ddot{q} = B^{-1}(q)\big(\tau - J^T(q)f - C(q,\dot{q})\dot{q} - D\dot{q} - g(q)\big ) 
  \f]
*/
{
   ColumnVector qpp(dof);
   qpp = 0.0;
   qpp = inertia(q).i()*(tau_cmd-torque(q, qp, qpp, Fext, Next));
   qpp.Release(); 
   return qpp;
}

ReturnMatrix Robot::torque(const ColumnVector & q, const ColumnVector & qp,
                           const ColumnVector & qpp)
/*!
  @brief Joint torque, without contact force, based on Recursive 
  Newton-Euler formulation.
*/
{
   ColumnVector Fext(3), Next(3);
   Fext = 0;
   Next = 0;
   return torque(q, qp, qpp, Fext, Next);
}

ReturnMatrix Robot::torque(const ColumnVector & q, const ColumnVector & qp,
                           const ColumnVector & qpp, const ColumnVector & Fext,
                           const ColumnVector & Next)
/*!
  @brief Joint torque based on Recursive Newton-Euler formulation.


  In order to apply the RNE as presented in Murray 86, 
  let us define the following variables 
  (referenced in the \f$i^{th}\f$ coordinate frame if applicable):

  \f$\sigma_i\f$ is the joint type; \f$\sigma_i = 1\f$ for a revolute
  joint and \f$\sigma_i = 0\f$ for a prismatic joint.

  \f$ z_0 = 
    \left [
      \begin{array}{ccc}
         0 & 0 & 1
      \end{array}
    \right ]^T\f$

  \f$p_i = 
    \left [
      \begin{array}{ccc}
      a_i & d_i \sin \alpha_i & d_i \cos \alpha_i
      \end{array}
    \right ]^T\f$ is the position of the \f$i^{th}\f$ with respect to the \f$i-1^{th}\f$ frame.

    Forward Iterations for \f$i=1, 2, \ldots, n\f$. 
    Initialize: \f$\omega_0 = \dot{\omega}_0 = 0\f$ and \f$\dot{v}_0 = - g\f$.
    \f[
    \omega_i = R_i^T [\omega_{i-1} + \sigma_i z_0 \dot{\theta}_i ] 
    \f]
    \f[
    \dot{\omega}_i = R_i^T  \{ \dot{\omega}_{i-1} + 
    \sigma_i [z_0 \ddot{\theta}_i + \omega_{i-1} \times (z_0 \dot{\theta}_i )] \} 
    \f]
    \f[
    \dot{v}_i = R_i^T  \{ \dot{v}_{i-1} + 
    (1 -\sigma_i) [z_0 \ddot{d}_i + 2 \omega_{i-1} \times (z_0 \dot{d}_i )] \} 
    + \dot{\omega}_i \times p_i + \omega_i \times ( \omega_i \times p_i)
    \f]

    Backward Iterations for \f$i=n, n-1, \ldots, 1\f$. 
    Initialize: $f_{n+1} = n_{n+1} = 0$.
    \f[
    \dot{v}_{ci} = v_i + \omega_i \times r_i 
    + \omega_i \times (\omega_i \times r_i) 
    \f]
    \f[
    F_i = m_i \dot{v}_{ci} 
    \f]
    \f[
    N_i = I_{ci} \dot{\omega}_i + \omega_i \times (I_{ci} \omega_i)
    \f]
    \f[
    f_i = R_{i+1} [ f_{i+1} ] + F_{i} 
    \f]
    \f[
    n_i = R_{i+1} [ n_{i+1} ]  + p_{i} \times f_{i} 
    + N_{i} + r_{i} \times F_{i}
    \f]
    \f[
    \tau_i = \sigma_i n_i^T (R_i^T z_0) 
    + (1 - \sigma_i) f_i^T (R_i^T z_0)
    \f]
*/
{
   int i;
   ColumnVector ltorque(dof);
   Matrix Rt, temp;
   if(q.Nrows() != dof) error("q has wrong dimension");
   if(qp.Nrows() != dof) error("qp has wrong dimension");
   if(qpp.Nrows() != dof) error("qpp has wrong dimension");
   set_q(q);
   set_qp(qp);

   vp[0] = gravity;
   for(i = 1; i <= dof; i++) {
      Rt = links[i].R.t();
      if(links[i].get_joint_type() == 0) {
         w[i] = Rt*(w[i-1] + z0*qp(i));
         wp[i] = Rt*(wp[i-1] + z0*qpp(i)
                     + CrossProduct(w[i-1],z0*qp(i)));
         vp[i] = CrossProduct(wp[i],p[i])
                 + CrossProduct(w[i],CrossProduct(w[i],p[i]))
                 + Rt*(vp[i-1]);
      } else {
         w[i] = Rt*w[i-1];
         wp[i] = Rt*wp[i-1];
         vp[i] = Rt*(vp[i-1] + z0*qpp(i))
                 + 2.0*CrossProduct(w[i],Rt*z0*qp(i))
                 + CrossProduct(wp[i],p[i])
                 + CrossProduct(w[i],CrossProduct(w[i],p[i]));
      }
      a[i] = CrossProduct(wp[i],links[i].r)
             + CrossProduct(w[i],CrossProduct(w[i],links[i].r))
             + vp[i];
   }

   for(i = dof; i >= 1; i--) {
      F[i] =  a[i] * links[i].m;
      N[i] = links[i].I*wp[i] + CrossProduct(w[i],links[i].I*w[i]);
      if(i == dof) {
         f[i] = F[i] + Fext;
         n[i] = CrossProduct(p[i],f[i])
                + CrossProduct(links[i].r,F[i]) + N[i] + Next;
      } else {
         f[i] = links[i+1].R*f[i+1] + F[i];
         n[i] = links[i+1].R*n[i+1] + CrossProduct(p[i],f[i])
                + CrossProduct(links[i].r,F[i]) + N[i];
      }
      if(links[i].get_joint_type() == 0)
         temp = ((z0.t()*links[i].R)*n[i]);
      else
         temp = ((z0.t()*links[i].R)*f[i]);
      ltorque(i) = temp(1,1)
                   + links[i].Im*links[i].Gr*links[i].Gr*qpp(i)
                   + links[i].Gr*(links[i].Gr*links[i].B*qp(i) + links[i].Cf*sign(qp(i)));
   }

   ltorque.Release(); return ltorque;
}

ReturnMatrix Robot::torque_novelocity(const ColumnVector & qpp)
/*!
  @brief Joint torque. when joint velocity is 0, based on Recursive 
  Newton-Euler formulation.
*/
{
   int i;
   ColumnVector ltorque(dof);
   Matrix Rt, temp;
   if(qpp.Nrows() != dof) error("qpp has wrong dimension");

   vp[0] = 0.0;
   for(i = 1; i <= dof; i++) {
      Rt = links[i].R.t();
      if(links[i].get_joint_type() == 0) {
         wp[i] = Rt*(wp[i-1] + z0*qpp(i));
         vp[i] = CrossProduct(wp[i],p[i])
                 + Rt*(vp[i-1]);
      } else {
         wp[i] = Rt*wp[i-1];
         vp[i] = Rt*(vp[i-1] + z0*qpp(i))
                 + CrossProduct(wp[i],p[i]);
      }
      a[i] = CrossProduct(wp[i],links[i].r) + vp[i];
   }

   for(i = dof; i >= 1; i--) {
      F[i] = a[i] * links[i].m;
      N[i] = links[i].I*wp[i];
      if(i == dof) {
         f_nv[i] = F[i];
         n_nv[i] = CrossProduct(p[i],f_nv[i])
                   + CrossProduct(links[i].r,F[i]) + N[i];
      } else {
         f_nv[i] = links[i+1].R*f_nv[i+1] + F[i];
         n_nv[i] = links[i+1].R*n_nv[i+1] + CrossProduct(p[i],f_nv[i])
                   + CrossProduct(links[i].r,F[i]) + N[i];
      }
      if(links[i].get_joint_type() == 0)
         temp = ((z0.t()*links[i].R)*n_nv[i]);
      else
         temp = ((z0.t()*links[i].R)*f_nv[i]);
      ltorque(i) = temp(1,1) + links[i].Im*links[i].Gr*links[i].Gr*qpp(i);

   }

   ltorque.Release(); return ltorque;
}

ReturnMatrix Robot::G()
//! @brief Joint torque due to gravity based on Recursive Newton-Euler formulation.
{
   int i;
   ColumnVector ltorque(dof);
   Matrix Rt, temp;

   vp[0] = gravity;
   for(i = 1; i <= dof; i++) {
      Rt = links[i].R.t();
      if(links[i].get_joint_type() == 0)
         vp[i] = Rt*(vp[i-1]);
      else
         vp[i] = Rt*vp[i-1];

      a[i] = vp[i];
   }

   for(i = dof; i >= 1; i--) {
      F[i] =  a[i] * links[i].m;
      if(i == dof) {
         f[i] = F[i];
         n[i] = CrossProduct(p[i],f[i])
                + CrossProduct(links[i].r,F[i]);
      } else {
         f[i] = links[i+1].R*f[i+1] + F[i];
         n[i] = links[i+1].R*n[i+1] + CrossProduct(p[i],f[i])
                + CrossProduct(links[i].r,F[i]);
      }
      if(links[i].get_joint_type() == 0)
         temp = ((z0.t()*links[i].R)*n[i]);
      else
         temp = ((z0.t()*links[i].R)*f[i]);
      ltorque(i) = temp(1,1);
   }

   ltorque.Release(); return ltorque;
}

ReturnMatrix Robot::C(const ColumnVector & qp)
//! @brief Joint torque due to centrifugal and Corriolis based on Recursive Newton-Euler formulation.
{
   int i;
   ColumnVector ltorque(dof);
   Matrix Rt, temp;
   if(qp.Nrows() != dof) error("qp has wrong dimension");

   vp[0]=0;
   for(i = 1; i <= dof; i++) {
      Rt = links[i].R.t();
      if(links[i].get_joint_type() == 0) {
         w[i] = Rt*(w[i-1] + z0*qp(i));
         wp[i] = Rt*(wp[i-1] + CrossProduct(w[i-1],z0*qp(i)));
         vp[i] = CrossProduct(wp[i],p[i])
                 + CrossProduct(w[i],CrossProduct(w[i],p[i]))
                 + Rt*(vp[i-1]);
      } else {
         w[i] = Rt*w[i-1];
         wp[i] = Rt*wp[i-1];
         vp[i] = Rt*vp[i-1] + 2.0*CrossProduct(w[i],Rt*z0*qp(i))
                 + CrossProduct(wp[i],p[i])
                 + CrossProduct(w[i],CrossProduct(w[i],p[i]));
      }
      a[i] = CrossProduct(wp[i],links[i].r)
             + CrossProduct(w[i],CrossProduct(w[i],links[i].r))
             + vp[i];
   }

   for(i = dof; i >= 1; i--) {
      F[i] =  a[i] * links[i].m;
      N[i] = links[i].I*wp[i] + CrossProduct(w[i],links[i].I*w[i]);
      if(i == dof) {
         f[i] = F[i];
         n[i] = CrossProduct(p[i],f[i])
                + CrossProduct(links[i].r,F[i]) + N[i];
      } else {
         f[i] = links[i+1].R*f[i+1] + F[i];
         n[i] = links[i+1].R*n[i+1] + CrossProduct(p[i],f[i])
                + CrossProduct(links[i].r,F[i]) + N[i];
      }
      if(links[i].get_joint_type() == 0)
         temp = ((z0.t()*links[i].R)*n[i]);
      else
         temp = ((z0.t()*links[i].R)*f[i]);
      ltorque(i) = temp(1,1)
                   + links[i].Gr*(links[i].Gr*links[i].B*qp(i) + links[i].Cf*sign(qp(i)));
   }

   ltorque.Release(); return ltorque;
}

ReturnMatrix mRobot::torque(const ColumnVector & q, const ColumnVector & qp,
                            const ColumnVector & qpp)
//! @brief Joint torque, without contact force, based on Recursive Newton-Euler formulation.
{
   ColumnVector Fext(3), Next(3);
   Fext = 0;
   Next = 0;
   return torque(q, qp, qpp, Fext, Next);
}

ReturnMatrix mRobot::torque(const ColumnVector & q, const ColumnVector & qp,
                            const ColumnVector & qpp, const ColumnVector & Fext_,
                            const ColumnVector & Next_)
/*!
  @brief Joint torque based on Recursive Newton-Euler formulation.


  In order to apply the RNE, let us define the following variables 
  (referenced in the \f$i^{th}\f$ coordinate frame if applicable):

  \f$\sigma_i\f$ is the joint type; \f$\sigma_i = 1\f$ for a revolute
  joint and \f$\sigma_i = 0\f$ for a prismatic joint.

  \f$ z_0 = 
    \left [
      \begin{array}{ccc}
         0 & 0 & 1
      \end{array}
    \right ]^T\f$

  \f$p_i =
    \left [
      \begin{array}{ccc}
      a_{i-1} & -d_i sin \alpha_{i-1} & d_i cos \alpha_{i-1}
      \end{array}
    \right ]^T\f$ is the position of the $i^{th}$ with respect to the $i-1^{th}$ frame.

  Forward Iterations for \f$i=1, 2, \ldots, n\f$.  Initialize: 
  \f$\omega_0 = \dot{\omega}_0 = 0$ and $\dot{v}_0 = - g\f$. 

  \f[
  \omega_i = R_i^T\omega_{i-1} + \sigma_i z_0\dot{\theta_i} 
  \f]
  \f[
  \dot{\omega}_i = R_i^T\dot{\omega}_{i-1} + \sigma_i
  R_i^T\omega_{i-1}\times z_0 \dot{\theta}_i + \sigma_iz_0\ddot{\theta}_i
  \f]
  \f[
  \dot{v}_i = R_i^T(\dot{\omega}_{i-1}\times p_i +
  \omega_{i-1}\times(\omega_{i-1}\times p_i) + \dot{v}_{i-1})
  + (1 - \sigma_i)(2\omega_i\times z_0 dot{d}_i + z_0\ddot{d}_i)
  \f]

  Backward Iterations for \f$i=n, n-1, \ldots, 1\f$. Initialize: \f$f_{n+1} = n_{n+1} = 0\f$.

  \f[
  \dot{v}_{ci} = \dot{\omega}_i\times r_i + 
  \omega_i\times(\omega_i\times r_i) + v_i 
  \f]
  \f[
  F_i = m_i \dot{v}_{ci} 
  \f]
  \f[
  N_i = I_{ci}\ddot{\omega}_i\ + \omega_i \times I_{ci}\omega_i 
  \f]
  \f[
  f_i = R_{i+1}f_{i+1} + F_i
  \f]
  \f[
  n_i = N_i + R_{i+1} n_{i+1} + r_i \times F_i + p_{i+1}\times R_{i+1}f_{i+1}
  \f]
  \f[
  \tau_i = \sigma_i n_i^T z_0 + (1 - \sigma_i) f_i^T z_0
  \f]
*/
{
   int i;
   ColumnVector ltorque(dof);
   Matrix Rt, temp;
   if(q.Nrows() != dof) error("q has wrong dimension");
   if(qp.Nrows() != dof) error("qp has wrong dimension");
   if(qpp.Nrows() != dof) error("qpp has wrong dimension");
   set_q(q);
   set_qp(qp);

   vp[0] = gravity;
   for(i = 1; i <= dof; i++) {
      Rt = links[i].R.t();
      if(links[i].get_joint_type() == 0) {
         w[i] = Rt*w[i-1] + z0*qp(i);
         wp[i] = Rt*wp[i-1] + CrossProduct(Rt*w[i-1],z0*qp(i))
                 + z0*qpp(i);
         vp[i] = Rt*(CrossProduct(wp[i-1],p[i])
                     + CrossProduct(w[i-1],CrossProduct(w[i-1],p[i]))
                     + vp[i-1]);
      } else {
         w[i] = Rt*w[i-1];
         wp[i] = Rt*wp[i-1];
         vp[i] = Rt*(vp[i-1] + CrossProduct(wp[i-1],p[i])
                     + CrossProduct(w[i-1],CrossProduct(w[i-1],p[i])))
                 + z0*qpp(i)+ 2.0*CrossProduct(w[i],z0*qp(i));
      }
      a[i] = CrossProduct(wp[i],links[i].r)
             + CrossProduct(w[i],CrossProduct(w[i],links[i].r))
             + vp[i];
   }

   // Load on last link
   ColumnVector Fext(3), Next(3);
   if(fix) // Last link is fix
   {
      Fext = links[dof+fix].R*Fext_;
      Next = links[dof+fix].R*Next_;
   }
   else
   {
      Fext = Fext_;
      Next = Next_;
   }

   for(i = dof; i >= 1; i--) {
      F[i] =  a[i] * links[i].m;
      N[i] = links[i].I*wp[i] + CrossProduct(w[i],links[i].I*w[i]);
      if(i == dof) {
         f[i] = F[i] + Fext;
         n[i] = CrossProduct(links[i].r,F[i]) + N[i] + Next;
      } else {
         f[i] = links[i+1].R*f[i+1] + F[i];
         n[i] = links[i+1].R*n[i+1] + CrossProduct(p[i+1],links[i+1].R*f[i+1])
                + CrossProduct(links[i].r,F[i]) + N[i];
      }
      if(links[i].get_joint_type() == 0)
         temp = (z0.t()*n[i]);
      else
         temp = (z0.t()*f[i]);
      ltorque(i) = temp(1,1)
                   + links[i].Im*links[i].Gr*links[i].Gr*qpp(i)
                   + links[i].Gr*(links[i].Gr*links[i].B*qp(i) + links[i].Cf*sign(qp(i)));
   }

   ltorque.Release(); return ltorque;
}

ReturnMatrix mRobot::torque_novelocity(const ColumnVector & qpp)
//! @brief Joint torque. when joint velocity is 0, based on Recursive Newton-Euler formulation.
{
   int i;
   ColumnVector ltorque(dof);
   Matrix Rt, temp;
   if(qpp.Ncols() != 1 || qpp.Nrows() != dof) error("qpp has wrong dimension");

   vp[0] = 0.0;
   for(i = 1; i <= dof; i++) {
      Rt = links[i].R.t();
      if(links[i].get_joint_type() == 0) {
         wp[i] = Rt*wp[i-1] + z0*qpp(i);
         vp[i] = Rt*(CrossProduct(wp[i-1],p[i]) + vp[i-1]);
      } else {
         wp[i] = Rt*wp[i-1];
         vp[i] = Rt*(vp[i-1] + CrossProduct(wp[i-1],p[i]))
                 + z0*qpp(i);
      }
      a[i] = CrossProduct(wp[i],links[i].r) + vp[i];
   }

   for(i = dof; i >= 1; i--) {
      F[i] =  a[i] * links[i].m;
      N[i] = links[i].I*wp[i];
      if(i == dof) {
         f_nv[i] = F[i];
         n_nv[i] = CrossProduct(links[i].r,F[i]) + N[i];
      } else {
         f_nv[i] = links[i+1].R*f_nv[i+1] + F[i];
         n_nv[i] = links[i+1].R*n_nv[i+1] + CrossProduct(p[i+1],links[i+1].R*f_nv[i+1])
                   + CrossProduct(links[i].r,F[i]) + N[i];
      }

      if(links[i].get_joint_type() == 0)
         temp = (z0.t()*n_nv[i]);
      else
         temp = (z0.t()*f_nv[i]);
      ltorque(i) = temp(1,1) + links[i].Im*links[i].Gr*links[i].Gr*qpp(i);
   }

   ltorque.Release(); return ltorque;
}

ReturnMatrix mRobot::G()
//! @brief Joint torque due to gravity based on Recursive Newton-Euler formulation.
{
   int i;
   ColumnVector ltorque(dof);
   Matrix Rt, temp;

   vp[0] = gravity;
   for(i = 1; i <= dof; i++) {
      Rt = links[i].R.t();
      if(links[i].get_joint_type() == 0)
         vp[i] = Rt*vp[i-1];
      else
         vp[i] = Rt*vp[i-1];
      a[i] = vp[i];
   }

   for(i = dof; i >= 1; i--) {
      F[i] =  a[i] * links[i].m;
      if(i == dof) {
         f[i] = F[i];
         n[i] = CrossProduct(links[i].r,F[i]);
      } else {
         f[i] = links[i+1].R*f[i+1] + F[i];
         n[i] = links[i+1].R*n[i+1] + CrossProduct(p[i+1],links[i+1].R*f[i+1])
                + CrossProduct(links[i].r,F[i]);
      }
      if(links[i].get_joint_type() == 0)
         temp = (z0.t()*n[i]);
      else
         temp = (z0.t()*f[i]);
      ltorque(i) = temp(1,1);
   }

   ltorque.Release(); return ltorque;
}

ReturnMatrix mRobot::C(const ColumnVector & qp)
//! @brief Joint torque due to centrifugal and Corriolis based on Recursive Newton-Euler formulation.  
{
   int i;
   ColumnVector ltorque(dof);
   Matrix Rt, temp;
   if(qp.Nrows() != dof) error("qp has wrong dimension");

   vp[0] = 0;
   for(i = 1; i <= dof; i++) {
      Rt = links[i].R.t();
      if(links[i].get_joint_type() == 0) {
         w[i] = Rt*w[i-1] + z0*qp(i);
         wp[i] = Rt*wp[i-1] + CrossProduct(Rt*w[i-1],z0*qp(i));
         vp[i] = Rt*(CrossProduct(wp[i-1],p[i])
                     + CrossProduct(w[i-1],CrossProduct(w[i-1],p[i]))
                     + vp[i-1]);
      } else {
         w[i] = Rt*w[i-1];
         wp[i] = Rt*wp[i-1];
         vp[i] = Rt*(vp[i-1] + CrossProduct(wp[i-1],p[i])
                     + CrossProduct(w[i-1],CrossProduct(w[i-1],p[i])))
                 +  2.0*CrossProduct(w[i],z0*qp(i));
      }
      a[i] = CrossProduct(wp[i],links[i].r)
             + CrossProduct(w[i],CrossProduct(w[i],links[i].r))
             + vp[i];
   }

   for(i = dof; i >= 1; i--) {
      F[i] =  a[i] * links[i].m;
      N[i] = links[i].I*wp[i] + CrossProduct(w[i],links[i].I*w[i]);
      if(i == dof) {
         f[i] = F[i];
         n[i] = CrossProduct(links[i].r,F[i]) + N[i];
      } else {
         f[i] = links[i+1].R*f[i+1] + F[i];
         n[i] = links[i+1].R*n[i+1] + CrossProduct(p[i+1],links[i+1].R*f[i+1])
                + CrossProduct(links[i].r,F[i]) + N[i];
      }
      if(links[i].get_joint_type() == 0)
         temp = (z0.t()*n[i]);
      else
         temp = (z0.t()*f[i]);
      ltorque(i) = temp(1,1)
                   + links[i].Gr*(links[i].Gr*links[i].B*qp(i) + links[i].Cf*sign(qp(i)));
   }

   ltorque.Release(); return ltorque;
}

ReturnMatrix mRobot_min_para::torque(const ColumnVector & q, const ColumnVector & qp,
                                     const ColumnVector & qpp)
//! @brief Joint torque without contact force based on Recursive Newton-Euler formulation.
{
   ColumnVector Fext(3), Next(3);
   Fext = 0;
   Next = 0;
   return torque(q, qp, qpp, Fext, Next);
}

ReturnMatrix mRobot_min_para::torque(const ColumnVector & q, const ColumnVector & qp,
                                     const ColumnVector & qpp, const ColumnVector & Fext_,
                                     const ColumnVector & Next_)
/*!
  @brief Joint torque based on Recursive Newton-Euler formulation.

  See ReturnMatrix mRobot::torque(const ColumnVector & q, const ColumnVector & qp,
                                  const ColumnVector & qpp, const ColumnVector & Fext,
                                  const ColumnVector & Next)
  for the Recursive Newton-Euler formulation.
*/
{
   int i;
   ColumnVector ltorque(dof);
   Matrix Rt, temp;
   if(q.Nrows() != dof) error("q has wrong dimension");
   if(qp.Nrows() != dof) error("qp has wrong dimension");
   if(qpp.Nrows() != dof) error("qpp has wrong dimension");
   set_q(q);
   set_qp(qp);

   vp[0] = gravity;
   for(i = 1; i <= dof; i++) {
      Rt = links[i].R.t();
      if(links[i].get_joint_type() == 0)
      {
         w[i] = Rt*w[i-1] + z0*qp(i);
         wp[i] = Rt*(wp[i-1] + CrossProduct(w[i-1],z0*qp(i)))
                 + z0*qpp(i);
         vp[i] = Rt*(CrossProduct(wp[i-1],p[i])
                     + CrossProduct(w[i-1],CrossProduct(w[i-1],p[i]))
                     + vp[i-1]);
      }
      else
      {
         w[i] = Rt*w[i-1];
         wp[i] = Rt*wp[i-1];
         vp[i] = Rt*(vp[i-1] + CrossProduct(wp[i-1],p[i])
                     + CrossProduct(w[i-1],CrossProduct(w[i-1],p[i])))
                 + z0*qpp(i)+ 2.0*CrossProduct(w[i],z0*qp(i));
      }
   }

   ColumnVector Fext(3), Next(3);
   if(fix)
   {
      Fext = links[dof+fix].R*Fext_;
      Next = links[dof+fix].R*Next_;
   }
   else
   {
      Fext = Fext_;
      Next = Next_;
   }

   for(i = dof; i >= 1; i--)
   {
      F[i] = vp[i]*links[i].m + CrossProduct(wp[i], links[i].mc) +
             CrossProduct(w[i], CrossProduct(w[i], links[i].mc));
      N[i] = links[i].I*wp[i] + CrossProduct(w[i],links[i].I*w[i]) +
             CrossProduct(-vp[i], links[i].mc);
      if(i == dof)
      {
         f[i] = F[i] + Fext;
         n[i] = N[i] + Next;
      }
      else
      {
         f[i] = links[i+1].R*f[i+1] + F[i];
         n[i] = links[i+1].R*n[i+1] + CrossProduct(p[i+1],links[i+1].R*f[i+1]) + N[i];
      }

      if(links[i].get_joint_type() == 0)
         temp = (z0.t()*n[i]);
      else
         temp = (z0.t()*f[i]);
      ltorque(i) = temp(1,1)
                   + links[i].Im*links[i].Gr*links[i].Gr*qpp(i)
                   + links[i].Gr*(links[i].Gr*links[i].B*qp(i) + links[i].Cf*sign(qp(i)));
   }

   ltorque.Release(); return ltorque;
}


ReturnMatrix mRobot_min_para::torque_novelocity(const ColumnVector & qpp)
//! @brief Joint torque. when joint velocity is 0, based on Recursive Newton-Euler formulation.
{
   int i;
   ColumnVector ltorque(dof);
   Matrix Rt, temp;
   if(qpp.Ncols() != 1 || qpp.Nrows() != dof) error("qpp has wrong dimension");

   vp[0] = 0.0;
   for(i = 1; i <= dof; i++)
   {
      Rt = links[i].R.t();
      if(links[i].get_joint_type() == 0)
      {
         wp[i] = Rt*wp[i-1] + z0*qpp(i);
         vp[i] = Rt*(CrossProduct(wp[i-1],p[i]) + vp[i-1]);
      }
      else
      {
         wp[i] = Rt*wp[i-1];
         vp[i] = Rt*(vp[i-1] + CrossProduct(wp[i-1],p[i]))
                 + z0*qpp(i);
      }
   }

   for(i = dof; i >= 1; i--)
   {
      F[i] = vp[i]*links[i].m + CrossProduct(wp[i], links[i].mc);
      N[i] = links[i].I*wp[i] + CrossProduct(-vp[i], links[i].mc);
      if(i == dof)
      {
         f_nv[i] = F[i];
         n_nv[i] = N[i];
      }
      else
      {
         f_nv[i] = links[i+1].R*f_nv[i+1] + F[i];
         n_nv[i] = links[i+1].R*n_nv[i+1] + CrossProduct(p[i+1],links[i+1].R*f_nv[i+1]) + N[i];
      }

      if(links[i].get_joint_type() == 0)
         temp = (z0.t()*n_nv[i]);
      else
         temp = (z0.t()*f_nv[i]);
      ltorque(i) = temp(1,1) + links[i].Im*links[i].Gr*links[i].Gr*qpp(i);
   }

   ltorque.Release(); return ltorque;
}

ReturnMatrix mRobot_min_para::G()
//! @brief Joint torque due to gravity based on Recursive Newton-Euler formulation.
{
   int i;
   ColumnVector ltorque(dof);
   Matrix Rt, temp;

   vp[0] = gravity;
   for(i = 1; i <= dof; i++) {
      Rt = links[i].R.t();
      if(links[i].get_joint_type() == 0)
         vp[i] = Rt*vp[i-1];
      else
         vp[i] = Rt*vp[i-1];
   }

   for(i = dof; i >= 1; i--)
   {
      F[i] = vp[i]*links[i].m;
      N[i] = CrossProduct(-vp[i], links[i].mc);
      if(i == dof)
      {
         f[i] = F[i];
         n[i] = N[i];
      }
      else
      {
         f[i] = links[i+1].R*f[i+1] + F[i];
         n[i] = links[i+1].R*n[i+1] + CrossProduct(p[i+1],links[i+1].R*f[i+1]) + N[i];
      }

      if(links[i].get_joint_type() == 0)
         temp = (z0.t()*n[i]);
      else
         temp = (z0.t()*f[i]);
      ltorque(i) = temp(1,1);
   }

   ltorque.Release(); return ltorque;
}

ReturnMatrix mRobot_min_para::C(const ColumnVector & qp)
//! @brief Joint torque due to centrifugal and Corriolis based on Recursive Newton-Euler formulation.  
{
   int i;
   ColumnVector ltorque(dof);
   Matrix Rt, temp;
   if(qp.Nrows() != dof) error("qp has wrong dimension");
   set_qp(qp);

   vp[0] = 0;
   for(i = 1; i <= dof; i++) {
      Rt = links[i].R.t();
      if(links[i].get_joint_type() == 0)
      {
         w[i] = Rt*w[i-1] + z0*qp(i);
         wp[i] = Rt*(wp[i-1] + CrossProduct(w[i-1],z0*qp(i)));
         vp[i] = Rt*(CrossProduct(wp[i-1],p[i])
                     + CrossProduct(w[i-1],CrossProduct(w[i-1],p[i]))
                     + vp[i-1]);
      }
      else
      {
         w[i] = Rt*w[i-1];
         wp[i] = Rt*wp[i-1];
         vp[i] = Rt*(vp[i-1] + CrossProduct(wp[i-1],p[i])
                     + CrossProduct(w[i-1],CrossProduct(w[i-1],p[i])))
                 + 2.0*CrossProduct(w[i],z0*qp(i));
      }
   }

   for(i = dof; i >= 1; i--)
   {
      F[i] = vp[i]*links[i].m + CrossProduct(wp[i], links[i].mc) +
             CrossProduct(w[i], CrossProduct(w[i], links[i].mc));
      N[i] = links[i].I*wp[i] + CrossProduct(w[i],links[i].I*w[i]) +
             CrossProduct(-vp[i], links[i].mc);
      if(i == dof)
      {
         f[i] = F[i];
         n[i] = N[i];
      }
      else
      {
         f[i] = links[i+1].R*f[i+1] + F[i];
         n[i] = links[i+1].R*n[i+1] + CrossProduct(p[i+1],links[i+1].R*f[i+1]) + N[i];
      }

      if(links[i].get_joint_type() == 0)
         temp = (z0.t()*n[i]);
      else
         temp = (z0.t()*f[i]);
      ltorque(i) = temp(1,1)
                   + links[i].Gr*(links[i].Gr*links[i].B*qp(i) + links[i].Cf*sign(qp(i)));
   }

   ltorque.Release(); return ltorque;
}

#ifdef use_namespace
}
#endif

