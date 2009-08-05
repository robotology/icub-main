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

2005/06/10: Etienne Lachance
    -The desired joint acceleration was missing in the computed torque method.

2005/11/06: Etienne Lachance
    - No need to provide a copy constructor and the assignment operator 
      (operator=) for Impedance, Resolved_acc, Computed_torque_method and
      Proportional_Derivative classes. Instead we use the one provide by the
      compiler.
-------------------------------------------------------------------------------
*/

/*!
  @file controller.cpp
  @brief Differents controllers class.
*/

//! @brief RCS/CVS version.
static const char rcsid[] = "$Id: controller.cpp,v 1.1 2007/07/24 16:03:09 amaldo Exp $";


#include "controller.h"

#ifdef use_namespace
namespace ROBOOP {
  using namespace NEWMAT;
#endif


Impedance::Impedance()
//! @brief Constructor.
{
   pc = ColumnVector(3); pc = 0;
   pcp = pc;
   pcpp = pc;
   pcpp_prev = pc;
   qc = Quaternion();
   qcp = qc;
   qcp_prev = qc;
   quat = qc;
   wc = pc;
   wcp = pc;
}

Impedance::Impedance(const Robot_basic & robot, const DiagonalMatrix & Mp_,
                     const DiagonalMatrix & Dp_, const DiagonalMatrix & Kp_,
                     const DiagonalMatrix & Mo_, const DiagonalMatrix & Do_, 
		     const DiagonalMatrix & Ko_)
//! @brief Constructor.
{
   set_Mp(Mp_);
   set_Dp(Dp_);
   set_Kp(Kp_);
   set_Mo(Mo_);
   set_Do(Do_);
   set_Ko(Ko_);

   pc = ColumnVector(3); pc = 0;
   pcp = pc;
   pcp_prev = pc;
   pcpp = pc;
   pcpp_prev = pc;
   qc = Quaternion();
   qcp = qc;
   qcp_prev = qc;
   quat = qc;
   wc = pc;
   wcp = pc;
   wcp_prev = pc;

   Matrix Rot;
   robot.kine(Rot, pc);
   qc = Quaternion(Rot);
}

short Impedance::set_Mp(const DiagonalMatrix & Mp_)
/*!
  @brief Assign the translational impedance inertia matrix \f$M_p\f$.
  @return short: 0 or WRONG_SIZE if the matrix is not \f$3\times 3\f$.
*/
{
   if(Mp_.Nrows() != 3)
   {
      Mp = DiagonalMatrix(3); Mp = 1;
      cerr << "Impedance::set_Mp: wrong size for input matrix Mp" << endl;
      return WRONG_SIZE;
   }

   Mp = Mp_;
   return 0;
}

short Impedance::set_Mp(const Real Mp_i, const short i)
/*!
  @brief Assign the translational impedance inertia term \f$M_p(i,i)\f$.
  @return short: 0 or WRONG_SIZE if the matrix is not \f$3\times 3\f$.
*/
{
   if( (i < 0) || (i > 3) )
   {
      cerr << "Impedance::set_Mp: index i out of bound" << endl;
      return WRONG_SIZE;
   }

   Mp(i) = Mp_i;
   return 0;
}

short Impedance::set_Dp(const DiagonalMatrix & Dp_)
/*!
  @brief Assign the translational impedance damping matrix \f$D_p\f$.
  @return short: 0 or WRONG_SIZE if the matrix is not \f$3\times 3\f$.
*/
{
   if(Dp_.Nrows() != 3)
   {
      Dp = DiagonalMatrix(3); Dp = 1;
      cerr << "Impedance::set_Dp: input matrix Dp of wrong size" << endl;
      return WRONG_SIZE;
   }

   Dp = Dp_;
   return 0;
}

short Impedance::set_Dp(const Real Dp_i, const short i)
/*!
  @brief Assign the translational impedance damping term \f$D_p(i,i)\f$.
  @return short: 0 or WRONG_SIZE if the matrix is not \f$3\times 3\f$.
*/
{
   if( (i < 0) || (i > 3) )
   {
      cerr << "Impedance::set_Dp: index i out of bound" << endl;
      return WRONG_SIZE;
   }

   Dp(i) = Dp_i;
   return 0;
}

short Impedance::set_Kp(const DiagonalMatrix & Kp_)
/*!
  @brief Assign the translational impedance stifness matrix \f$K_p\f$.
  @return short: 0 or WRONG_SIZE if the matrix is not \f$3\times 3\f$.
*/
{
   if(Kp_.Nrows() != 3)
   {
      Kp = DiagonalMatrix(3); Kp = 1;
      cerr << "Impedance::set_Kp: wrong size for input matrix Kp" << endl;
      return WRONG_SIZE;
   }

   Kp = Kp_;
   return 0;
}

short Impedance::set_Kp(const Real Kp_i, const short i)
/*!
  @brief Assign the translational impedance stifness term \f$K_p(i,i)\f$.
  @return short: 0 or WRONG_SIZE if the matrix is not \f$3\times 3\f$.
*/
{
   if( (i < 0) || (i > 3) )
   {
      cerr << "Impedance::set_Mp: index i out of bound" << endl;
      return WRONG_SIZE;
   }

   Kp(i) = Kp_i;
   return 0;
}

short Impedance::set_Mo(const DiagonalMatrix & Mo_)
/*!
  @brief Assign the rotational impedance inertia matrix \f$M_o\f$.
  @return short: 0 or WRONG_SIZE if the matrix is not \f$3\times 3\f$.
*/
{
   if(Mo_.Nrows() != 3)
   {
      Mo = DiagonalMatrix(3); Mo = 1;
      cerr << "Impedance::set_Mo: wrong size  input matrix Mo" << endl;
      return WRONG_SIZE;
   }

   Mo = Mo_;
   return 0;
}

short Impedance::set_Mo(const Real Mo_i, const short i)
/*!
  @brief Assign the rotational impedance inertia term \f$M_o(i,i)\f$.
  @return short: 0 or WRONG_SIZE if the matrix is not \f$3\times 3\f$.
*/
{
   if( (i < 0) || (i > 3) )
   {
      cerr << "Impedance::set_Mo: index i out of bound" << endl;
      return WRONG_SIZE;
   }

   Mo(i) = Mo_i;
   return 0;
}

short Impedance::set_Do(const DiagonalMatrix & Do_)
/*!
  @brief Assign the rotational impedance damping matrix \f$D_o\f$.
  @return short: 0 or WRONG_SIZE if the matrix is not \f$3\times 3\f$.
*/
{
   if( Do_.Nrows() != 3)
   {
      Do = DiagonalMatrix(3); Do = 1;
      cerr << "Impedance::set_Do: wrong size  input matrix Do" << endl;
      return WRONG_SIZE;
   }

   Do = Do_;
   return 0;
}

short Impedance::set_Do(const Real Do_i, const short i)
/*!
  @brief Assign the rotational impedance damping term \f$D_o(i,i)\f$.
  @return short: 0 or WRONG_SIZE if the matrix is not \f$3\times 3\f$.
*/
{
   if( (i < 0) || (i > 3) )
   {
      cerr << "Impedance::set_Do: index i out of bound" << endl;
      return WRONG_SIZE;
   }

   Do(i) = Do_i;
   return 0;
}

short Impedance::set_Ko(const DiagonalMatrix & Ko_)
/*!
  @brief Assign the rotational impedance stifness matrix \f$K_o\f$.
  @return short: 0 or WRONG_SIZE if the matrix is not \f$3\times 3\f$.
*/
{
   if(Ko_.Nrows() != 3)
   {
      Ko = DiagonalMatrix(3); Ko = 1;
      cerr << "Impedance::set_Ko: wrong size for  input matrix Ko" << endl;
      return WRONG_SIZE;
   }

   Ko = Ko_;
   return 0;
}

short Impedance::set_Ko(const Real Ko_i, const short i)
/*!
  @brief Assign the rotational impedance stifness term \f$K_o(i,i)\f$.
  @return short: 0 or WRONG_SIZE if the matrix is not \f$3\times 3\f$.
*/
{
   if( (i < 0) || (i > 3) )
   {
      cerr << "Impedance::set_Ko: index i out of bound" << endl;
      return WRONG_SIZE;
   }

   Ko(i) = Ko_i;
   return 0;
}

short Impedance::control(const ColumnVector & pdpp, const ColumnVector & pdp,
                         const ColumnVector & pd, const ColumnVector & wdp,
                         const ColumnVector & wd, const Quaternion & qd,
                         const ColumnVector & f, const ColumnVector & n,
                         const Real dt)
/*!
  @brief Generation of a compliance trajectory
  @param pdpp: desired end effector acceleration.
  @param pdp: desired end effector velocity.
  @param pd: desired end effector position.
  @param wdp: desired end effector angular acceleration.
  @param wd: desired end effector angular velocity.
  @param qd: desired quaternion.
  @param f: end effector contact force.
  @param n: end effector contact moment.
  @param dt: time frame.

  @return short: 0 or WRONG_SIZE if one of the vector input is not \f$3\times 1\f$.

  The translational and rotational impedance equations are integrated, with input 
  \f$f\f$ and \f$n\f$ to computed \f$\ddot{p}_c\f$ and \f$\dot{\omega}_c\f$, 
  \f$\dot{p}_c\f$ and \f$\omega_c\f$, and then \f$p_c\f$ and \f$q_c\f$. The compliant
  quaternion \f$q_c\f$ is obtained with the quaternion propagation equations (see 
  Quaternion class).

  The quaternion -q represents exactly the same rotation as the quaternion q. The 
  temporay quaternion, quat, is quatd plus a sign correction. It is customary to 
  choose the sign G on q1 so that q0.Gq1 >=0 (the angle between q0 ang Gq1 is acute). 
  This choice avoids extra spinning caused by the interpolated rotations.
*/
{
   if(pdpp.Nrows() !=3)
   {
      cerr << "Impedance::control: wrong size for input vector pdpp" << endl;
      return WRONG_SIZE;
   }
   if(pdp.Nrows() !=3)
   {
      cerr << "Impedance::control: wrong size for input vector pdp" << endl;
      return WRONG_SIZE;
   }
   if(pd.Nrows() != 3)
   {
      cerr << "Impedance::control: wrong size for input vector pd" << endl;
      return WRONG_SIZE;
   }
   if(wdp.Nrows() !=3)
   {
      cerr << "Impedance::control: wrong size for input vector wdp" << endl;
      return WRONG_SIZE;
   }
   if(wd.Nrows() !=3)
   {
      cerr << "Impedance::control: wrong size for input vector wd" << endl;
      return WRONG_SIZE;
   }
   if(f.Nrows() !=3)
   {
      cerr << "Impedance::control: wrong size for input vector f" << endl;
      return WRONG_SIZE;
   }
   if(n.Nrows() !=3)
   {
      cerr << "Impedance::control: wrong size for input vector f" << endl;
      return WRONG_SIZE;
   }

   static bool first=true;
   if(first)
   {
      qc = qd;
      qcp = qc.dot(wc, BASE_FRAME);
      qcp_prev = qcp;
      wc = wd;
      wcp = wdp;
      first = false;
   }
   if(qc.dot_prod(qd) < 0)
      quat = qd*(-1);
   else
      quat = qd;

   qcd = quat * qc.i();

   // Solving pcpp, pcp, pc with the translational impedance
   pcd = pc - pd;
   pcdp = pcp - pdp;
   //  pcpp = pdpp + Mp.i()*(f - Dp*pcdp - Kp*pcd - 2*qcd.s()*Km.t()*qcd.v()); // (21)
   pcpp = pdpp + Mp.i()*(f - Dp*pcdp - Kp*pcd);
   pcp = pcp_prev + Integ_Trap(pcpp, pcpp_prev, dt);
   pc = pc + Integ_Trap(pcp, pcp_prev, dt);

   // Solving wcp, wc, qc with the rotational impedance
   wcd = wc - wd;
   Ko_prime = 2*qcd.E(BASE_FRAME)*Ko;                                   //(23)
   //  Km_prime = (qcd.s()*qcd.E(BASE_FRAME) - qcd.v()*qcd.v().t())*Km;   // (24)
   //  wcp = wdp + Mo.i()*(n - Do*wcd - Ko_prime*qcd.v() - Km_prime*pcd); // (22)
   wcp = wdp + Mo.i()*(n - Do*wcd - Ko_prime*qcd.v()); // (22)
   wc = wc + Integ_Trap(wcp, wcp_prev, dt);
   qcp = qc.dot(wc, BASE_FRAME);
   Integ_quat(qcp, qcp_prev, qc, dt);

   return 0;
}

// ------------------------------------------------------------------------------------------------------
//  Position control based on resolved acceleration.
// ------------------------------------------------------------------------------------------------------

Resolved_acc::Resolved_acc(const short dof)
//! @brief Constructor.
{
   Kvp = Kpp = Kvo = Kpo = 0;
   zero3 = ColumnVector(3); zero3 = 0;
   p = zero3;
   pp = zero3;

   if(dof>0)
   {
       qp = ColumnVector(dof); qp = 0;
       qpp = qp;
   }

   quat_v_error = zero3;
   a = ColumnVector(6); a = 0;
   quat = Quaternion();
}

Resolved_acc::Resolved_acc(const Robot_basic & robot,
                           const Real Kvp_,
                           const Real Kpp_,
                           const Real Kvo_,
                           const Real Kpo_)
//! @brief Constructor.
{
   set_Kvp(Kvp_);
   set_Kpp(Kpp_);
   set_Kvo(Kvo_);
   set_Kpo(Kpo_);
   zero3 = ColumnVector(3); zero3 = 0;
   qp = ColumnVector(robot.get_dof()); qp = 0;
   qpp = qp;
   a = ColumnVector(6); a = 0;
   p = zero3;
   pp = zero3;
   quat_v_error = zero3;
   quat = Quaternion();
}

void Resolved_acc::set_Kvp(const Real Kvp_)
//! @brief Assign the gain \f$k_{vp}\f$.
{
   Kvp = Kvp_;
}

void Resolved_acc::set_Kpp(const Real Kpp_)
//! @brief Assign the gain \f$k_{pp}\f$.
{
   Kpp = Kpp_;
}

void Resolved_acc::set_Kvo(const Real Kvo_)
//! @brief Assign the gain \f$k_{vo}\f$.
{
   Kvo = Kvo_;
}

void Resolved_acc::set_Kpo(const Real Kpo_)
//! @brief Assign the gain \f$k_{po}\f$.
{
   Kpo = Kpo_;
}

ReturnMatrix Resolved_acc::torque_cmd(Robot_basic & robot, const ColumnVector & pdpp,
                                      const ColumnVector & pdp, const ColumnVector & pd,
                                      const ColumnVector & wdp, const ColumnVector & wd,
                                      const Quaternion & quatd, const short link_pc,
                                      const Real dt)
/*!
  @brief Output torque.

  For more robustess the damped least squares inverse Jacobian is used instead of
  the inverse Jacobian.

  The quaternion -q represents exactly the same rotation as the quaternion q. The 
  temporay quaternion, quat, is quatd plus a sign correction. It is customary to 
  choose the sign G on q1 so that q0.Gq1 >=0 (the angle between q0 ang Gq1 is acute). 
  This choice avoids extra spinning caused by the interpolated rotations.
*/
{
   robot.kine_pd(Rot, p, pp, link_pc);

   Quaternion quate(Rot); // end effector orientation
   if(quate.dot_prod(quatd) < 0)
      quat = quatd*(-1);
   else
      quat = quatd;

   quat_v_error = quate.s()*quat.v() - quat.s()*quate.v() +
                  x_prod_matrix(quate.v())*quat.v();

   a.SubMatrix(1,3,1,1) = pdpp + Kvp*(pdp-pp) + Kpp*(pd-p);
   a.SubMatrix(4,6,1,1) = wdp + Kvo*(wd-Rot*robot.w[robot.get_dof()]) +
                          Kpo*quat_v_error;
   qp = robot.get_qp();
   //                          (eps, lamda_max)
   qpp = robot.jacobian_DLS_inv(0.015, 0.2)*(a - robot.jacobian_dot()*qp);
   return robot.torque(robot.get_q(),qp, qpp, zero3, zero3);
}


// ------------------------------------------------------------------------------
//  Position control based on Computed Torque Method
// ------------------------------------------------------------------------------

Computed_torque_method::Computed_torque_method(const short dof_)
//! @brief Constructor.
{
    dof = dof_;
    qpp = ColumnVector(dof); qpp = 0;
    q = qpp;
    qp = qpp;
    zero3 = ColumnVector(3); zero3 = 0;
}

Computed_torque_method::Computed_torque_method(const Robot_basic & robot,
					       const DiagonalMatrix & Kp,
					       const DiagonalMatrix & Kd)
//! @brief Constructor.
{
   dof = robot.get_dof();
   set_Kd(Kd);
   set_Kp(Kp);
   qpp = ColumnVector(dof); qpp = 0;
   q = qpp;
   qp = qpp;
   zero3 = ColumnVector(3); zero3 = 0;
}

ReturnMatrix Computed_torque_method::torque_cmd(Robot_basic & robot,
                                                const ColumnVector & qd,
                                                const ColumnVector & qpd, 
                                                const ColumnVector & qppd )
//! @brief Output torque.
{
   if(qd.Nrows() != dof)
   {
      ColumnVector tau(dof); tau = 0;
      cerr << "Computed_torque_methode::torque_cmd: wrong size for input vector qd" << endl;
      tau.Release();
      return tau;
   }
   if(qpd.Nrows() != dof)
   {
      ColumnVector tau(dof); tau = 0;
      cerr << "Computed_torque_methode::torque_cmd: wrong size for input vector qpd" << endl;
      tau.Release();
      return tau;
   }
   if(qppd.Nrows() != dof)
   {
      ColumnVector tau(dof); tau = 0;
      cerr << "Computed_torque_methode::torque_cmd: wrong size for input vector qppd" << endl;
      tau.Release();
      return tau;
   }

   q = robot.get_q();
   qp = robot.get_qp();
   qpp = Kp*(qd-q) + Kd*(qpd-qp) + qppd;
   return robot.torque(q, qp, qpp, zero3, zero3);
}

short Computed_torque_method::set_Kd(const DiagonalMatrix & Kd_)
/*!
  @brief Assign the velocity error gain matrix \f$K_d(i,i)\f$.
  @return short: 0 or WRONG_SIZE if the matrix is not \f$dof \times dof\f$.
*/
{
   if(Kd_.Nrows() != dof)
   {
      Kd = DiagonalMatrix(dof); Kd = 0;
      cerr << "Computed_torque_method::set_kd: wrong size for input matrix Kd." << endl;
      return WRONG_SIZE;
   }

   Kd = Kd_;
   return 0;
}

short Computed_torque_method::set_Kp(const DiagonalMatrix & Kp_)
/*!
  @brief Assign the position error gain matrix \f$K_p(i,i)\f$.
  @return short: 0 or WRONG_SIZE if the matrix is not \f$dof \times dof\f$.
*/
{
   if(Kp_.Nrows() != dof)
   {
      Kp = DiagonalMatrix(dof); Kp = 0;
      cerr << "Computed_torque_method::set_kp: wrong size for input matrix Kp." << endl;
      return WRONG_SIZE;
   }

   Kp = Kp_;
   return 0;
}

// ------------------------------------------------------------------------------
//  Position control based on Proportional_Derivative (PD)
// ------------------------------------------------------------------------------

Proportional_Derivative::Proportional_Derivative(const short dof_)
//! @brief Constructor.
{
  dof = dof_;
  q = ColumnVector(dof); q = 0;
  qp = q;
  zero3 = ColumnVector(3); zero3 = 0;
}

Proportional_Derivative::Proportional_Derivative(const Robot_basic & robot, 
						 const DiagonalMatrix & Kp, 
						 const DiagonalMatrix & Kd)
//! @brief Constructor.
{
   dof = robot.get_dof();
   set_Kp(Kp);
   set_Kd(Kd);
   q = ColumnVector(dof); q = 0;
   qp = q;
   tau = ColumnVector(dof); tau = 0;
   zero3 = ColumnVector(3); zero3 = 0;
}

ReturnMatrix Proportional_Derivative::torque_cmd(Robot_basic & robot, 
						 const ColumnVector & qd,
						 const ColumnVector & qpd)
//! @brief Output torque.
{
   if(qd.Nrows() != dof)
   {
      tau = 0;
      cerr << "Proportional_Derivative::torque_cmd: wrong size for input vector qd" << endl;
      return tau;
   }
   if(qpd.Nrows() != dof)
   {
      tau = 0;
      cerr << "Proportional_Derivative::torque_cmd: wrong size for input vector qpd" << endl;
      return tau;
   }

   q = robot.get_q();
   qp = robot.get_qp();
   tau = Kp*(qd-q) + Kd*(qpd-qp);

   return tau;
}

short Proportional_Derivative::set_Kd(const DiagonalMatrix & Kd_)
/*!
  @brief Assign the velocity error gain matrix \f$K_p(i,i)\f$.
  @return short: 0 or WRONG_SIZE if the matrix is not \f$dof \times dof\f$.
*/
{
   if(Kd_.Nrows() != dof)
   {
      Kd = DiagonalMatrix(dof); Kd = 0;
      cerr << "Proportional_Derivative::set_kd: wrong size for input matrix Kd." << endl;
      return WRONG_SIZE;
   }

   Kd = Kd_;
   return 0;
}

short Proportional_Derivative::set_Kp(const DiagonalMatrix & Kp_)
/*!
  @brief Assign the position error gain matrix \f$K_p(i,i)\f$.
  @return short: 0 or WRONG_SIZE if the matrix is not \f$dof \times dof\f$.
*/
{
   if(Kp_.Nrows() != dof)
   {
      Kp = DiagonalMatrix(dof); Kp = 0;
      cerr << "Computed_torque_method::set_kp: wrong size for input matrix Kp." << endl;
      return WRONG_SIZE;
   }

   Kp = Kp_;
   return 0;
}

#ifdef use_namespace
}
#endif
