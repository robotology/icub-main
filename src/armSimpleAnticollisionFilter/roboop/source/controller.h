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

2004/07/13: Ethan Tira-Thompson
    -Added support for newmat's use_namespace #define, using ROBOOP namespace

2005/06/10: Etienne Lachance
    -The desired joint acceleration was missing in the computed torque method.

2005/11/06: Etienne Lachance
    - No need to provide a copy constructor and the assignment operator 
      (operator=) for Impedance, Resolved_acc, Computed_torque_method and
      Proportional_Derivative classes. Instead we use the one provide by the
      compiler.
-------------------------------------------------------------------------------
*/

#ifndef CONTROLLER_H
#define CONTROLLER_H

/*!
  @file controller.h
  @brief Header file for controller class definitions.
*/

//! @brief RCS/CVS version.
static const char header_controller_rcsid[] = "$Id: controller.h,v 1.1 2007/07/24 16:03:09 amaldo Exp $";


#include "robot.h"
#include "quaternion.h"

#ifdef use_namespace
namespace ROBOOP {
  using namespace NEWMAT;
#endif

//! @brief Return value when input vectors or matrix don't have the right size.
#define WRONG_SIZE -1

/*!
  @class Impedance
  @brief Impedance controller class.

  The implemantation of the impedance controller is made of two section:
  the first one is the generation of a compliance trajectory and the second one 
  used a position controller to ensure the end effector follow the compliance
  trajectory (We recommended to used the resolve acceleration controller 
  scheme, implemented in the class Resolved_acc).

  This class generate a compliance path given by the translational
  and the rotational impedance.
  \f[
    M_p\ddot{\tilde{p}} + D_p\dot{\tilde{p}} + K_p\tilde{p} = f
  \f]
  \f[
    M_o\dot{\tilde{\omega}} + D_o\tilde{\omega} + K_o'\tilde{v} = n
  \f]
  where \f$\tilde{x} = x_c - x_d\f$ and \f$ v\f$ is the vector par of the 
  quaternion representing the orientation error between the compliant and desired
  frame. The orientation error can also be express by rotation matrix, 
  \f$ \tilde{R} = R_d^TR_c\f$. The quaternion mathematics are implemented in 
  the Quaternion class. The index \f$_c\f$ and \f$_d\f$ denote 
  the compliance and the desired respectively. 

  The impedance parameters \f$M_p\f$, \f$D_p\f$, \f$K_p\f$, \f$M_o\f$, \f$D_o\f$ 
  and \f$K_o\f$ are \f$3\times 3\f$ diagonal positive definite matrix
*/
class Impedance{
public:
   Impedance();
   Impedance(const Robot_basic & robot, const DiagonalMatrix & Mp_,
             const DiagonalMatrix & Dp_, const DiagonalMatrix & Kp_,
             const DiagonalMatrix & Mo_, const DiagonalMatrix & Do_, 
	     const DiagonalMatrix & Ko_);
   short set_Mp(const DiagonalMatrix & Mp_);
   short set_Mp(const Real MP_i, const short i);
   short set_Dp(const DiagonalMatrix & Dp_);
   short set_Dp(const Real Dp_i, const short i);
   short set_Kp(const DiagonalMatrix & Kp_);
   short set_Kp(const Real Kp_i, const short i);
   short set_Mo(const DiagonalMatrix & Mo_);
   short set_Mo(const Real Mo_i, const short i);
   short set_Do(const DiagonalMatrix & Do_);
   short set_Do(const Real Do_i, const short i);
   short set_Ko(const DiagonalMatrix & Ko_);
   short set_Ko(const Real Ko_i, const short i);
   short control(const ColumnVector & pdpp, const ColumnVector & pdp,
                 const ColumnVector & pd, const ColumnVector & wdp,
                 const ColumnVector & wd, const Quaternion & qd,
                 const ColumnVector & f, const ColumnVector & n,
                 const Real dt);

   Quaternion qc,          //!< Compliant frame quaternion.
              qcp,         //!< Compliant frame quaternion derivative.
              qcp_prev,    //!< Previous value of qcp.
              qcd,         //!< Orientation error (betweem compliant and desired frame) quaternion.
             quat;         //!< Temporary quaternion.
   ColumnVector pc,        //!< Compliant position.
                pcp,       //!< Compliant velocity.
                pcpp,      //!< Compliant acceleration.
                pcp_prev,  //!< Previous value of pcp.
                pcpp_prev, //!< Previous value of pcpp.
                pcd,       //!< Difference between pc and desired position.
                pcdp,      //!< Difference between pcp and desired velocity.
                wc,        //!< Compliant angular velocity.
                wcp,       //!< Compliant angular acceleration.
                wcp_prev,  //!< Previous value of wcp.
                wcd;       //!< Difference between wc and desired angular velocity.
private:
   DiagonalMatrix Mp,   //!< Translational impedance inertia matrix.
                  Dp,   //!< Translational impedance damping matrix.
                  Kp,   //!< Translational impedance stifness matrix.
                  Mo,   //!< Rotational impedance inertia matrix.
                  Do,   //!< Rotational impedance damping matrix.
                  Ko;   //!< Rotational impedance stifness matrix.
   Matrix Ko_prime;     //!< Modified rotational impedance stifness matrix.
};

/*!
  @class Resolved_acc
  @brief Resolved rate acceleration controller class.

  The dynamic model of a robot manipulator can be expressed in
  joint space as
  \f[
    B(q)\ddot{q} + C(q,\dot{q})\dot{q} + D\dot{q} + g(q) = \tau - J^T(q)f
  \f]
  According to the concept of inverse dynamics, the driving torques
  can be chosen as
  \f[
    \tau = B(q)J^{-1}(q)\big(a - \dot{J}(q,\dot{q})\dot{q}\big) +
    C(q,\dot{q})\dot{q} + D\dot{q} + g(q) - J^T(q)f
  \f]
  where \f$a\f$ is the a new control input defined by
  \f[
    a_p = \ddot{p}_d + k_{vp}\dot{\tilde{p}} + k_{pp}\tilde{p}
  \f]
  \f[
    a_o = \dot{\omega}_d + k_{vo}\dot{\tilde{\omega}} + k_{po}\tilde{v}
  \f]
  where \f$\tilde{x} = x_c - x_d\f$ and \f$ v\f$ is the vector par of the 
  quaternion representing the orientation error between the desired and end 
  effector frame. \f$k_{vp}\f$, \f$k_{pp}\f$, \f$k_{vo}\f$ and \f$k_{po}\f$ 
  are positive gains.
  
  Up to now this class has been tested only with a 6 dof robot.
*/
class Resolved_acc {
public:
   Resolved_acc(const short dof = 1);
   Resolved_acc(const Robot_basic & robot,
                const Real Kvp, const Real Kpp,
                const Real Kvo, const Real Kpo);
   void set_Kvp(const Real Kvp);
   void set_Kpp(const Real Kpp);
   void set_Kvo(const Real Kvo);
   void set_Kpo(const Real Kpo);

   ReturnMatrix torque_cmd(Robot_basic & robot, const ColumnVector & pdpp,
                           const ColumnVector & pdp, const ColumnVector & pd,
                           const ColumnVector & wdp, const ColumnVector & wd,
                           const Quaternion & qd, const short link_pc,
                           const Real dt);
private:
   double Kvp,                //!< Controller gains.
          Kpp, 
          Kvo, 
          Kpo; 
   Matrix Rot;                //!< Temporay rotation matrix.
   ColumnVector zero3,        //!< \f$3\times 1\f$ zero vector.
                qp,           //!< Robot joints velocity.
                qpp,          //!< Robot joints acceleration.
                a,            //!< Control input.
                p,            //!< End effector position.
                pp,           //!< End effector velocity.
                quat_v_error; //!< Vector part of error quaternion.
   Quaternion quat;           //!< Temporary quaternion.
};


/*!
  @class Computed_torque_method
  @brief Computer torque method controller class.

  The dynamic model of a robot manipulator can be expressed in
  joint space as
  \f[
    B(q)\ddot{q} + C(q,\dot{q})\dot{q} + D\dot{q} + g(q) = \tau - J^T(q)f
  \f]
  The driving torques can be expressed as
  \f[
    \tau = B(q)\big(\ddot{q}_d + K_d(\dot{q}_d-\dot{q}) 
    + K_p(q_d-q)\big) + C(q,\dot{q})\dot{q} + D\dot{q} + g(q) + J^T(q)f
  \f]
  where \f$K_p\f$, \f$K_d\f$ are diagonal positive definie matrix.
*/
class Computed_torque_method {
public:
   Computed_torque_method(const short dof = 1);
   Computed_torque_method(const Robot_basic & robot,
                          const DiagonalMatrix & Kp, const DiagonalMatrix & Kd);
   ReturnMatrix torque_cmd(Robot_basic & robot, const ColumnVector & qd,
                           const ColumnVector & qpd,
                           const ColumnVector & qppd);
   short set_Kd(const DiagonalMatrix & Kd);
   short set_Kp(const DiagonalMatrix & Kp);

private:
   int dof;            //!< Degree of freedom.
   ColumnVector q,     //!< Robot joints positions.
                qp,    //!< Robot joints velocity.
                qpp,   //!< Robot joints acceleration.
                zero3; //!< \f$3\times 1\f$ zero vector.
   DiagonalMatrix Kp,  //!< Position error gain.
                  Kd;  //!< Velocity error gain.
};


/*!
  @class Proportional_Derivative
  @brief Proportional derivative controller class

  The driving torques can be expressed as
  \f[
    \tau = K_p(q_d-q) + K_d(\dot{q}_d-q)
  \f]
  where \f$K_p\f$, \f$K_d\f$ are diagonal positive definie matrix.
*/
class Proportional_Derivative {
public:
   Proportional_Derivative(const short dof = 1);
   Proportional_Derivative(const Robot_basic & robot, const DiagonalMatrix & Kp, 
			   const DiagonalMatrix & Kd);
   ReturnMatrix torque_cmd(Robot_basic & robot, const ColumnVector & qd,
                           const ColumnVector & qpd);
   short set_Kd(const DiagonalMatrix & Kd);
   short set_Kp(const DiagonalMatrix & Kp);

private:
   int dof;            //!< Degree of freedom.
   ColumnVector q,     //!< Robot joints positions.
                qp,    //!< Robot joints velocity.
                qpp,   //!< Robot joints acceleration.
                tau,   //!< Output torque.
                zero3; //!< \f$3\times 1\f$ zero vector.
   DiagonalMatrix Kp,  //!< Position error gain.
                  Kd;  //!< Velocity error gain.
};

#ifdef use_namespace
}
#endif

#endif

