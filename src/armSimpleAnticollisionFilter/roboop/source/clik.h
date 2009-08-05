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

Reference:

CLIK: closed loop inverse kinematics

[1] S. Chiaverini, B. Siciliano, "The Unit Quaternion: A Useful Tool for
    Inverse Kinematics of Robot Manipulators", Systems Analysis, Modelling
    and Simulation, vol. 35, pp.45-60, 1999.

[2] F. Caccavale, S. Chiaverini, B. Siciliano, "Second-Order Kinematic Control 
    of Robot Manipulators with Jacobian Damped Least-Squares Inverse: Theory
    and Experiments", IEEE/ASME Trans on Mechatronics, vol 2, no. 3, 
    pp. 188-194, 1997.

[3] S. Chiaverini, B. Siciliano, "Review of the Damped Least-Squares Inverse 
    Kinematics with Experiments on an Industrial Robot Manipulator" IEEE Trans
    on Control Systems Technology, vol 2, no 2, june 1994

[4] C. Natale, "Quaternion-Based Representation of Rigid Bodies Orientation",
    PRISMA LAB, PRISMA Technical Report no. 97-05, Oct 1997.

The algorithme is based on [1], which is of first order.     

-------------------------------------------------------------------------------
Revision_history:

2004/07/01: Etienne Lachance
   -Added doxygen documentation.

2004/07/01: Ethan Tira-Thompson
    -Added support for newmat's use_namespace #define, using ROBOOP namespace

2006/01/21: Etienne Lachance
    -No need to include quaternion.h.
*/


#ifndef CLIK_H
#define CLIK_H

/*!
  @file clik.h
  @brief Header file for Clik class definitions.
*/

//! @brief RCS/CVS version.
static const char header_clik_rcsid[] = "$Id: clik.h,v 1.1 2007/07/24 16:03:09 amaldo Exp $";

#include "robot.h"

#ifdef use_namespace
namespace ROBOOP {
  using namespace NEWMAT;
#endif


//! @brief Using Clik under DH notation.
#define CLICK_DH           1
//! @brief Using Clik under modified DH notation.
#define CLICK_mDH          2
//! @brief Using Clik under modified DH notation with minimum intertial parameters
#define CLICK_mDH_min_para 3


//! @brief Handle Closed Loop Inverse Kinematics scheme.
class Clik {
public:
   Clik(){}
   Clik(const Robot & robot_, const DiagonalMatrix & Kp_, const DiagonalMatrix & Ko_,
        const Real eps_=0.04, const Real lambda_max_=0.04, const Real dt=1.0);
   Clik(const mRobot & mrobot_, const DiagonalMatrix & Kp_, const DiagonalMatrix & Ko_,
        const Real eps_=0.04, const Real lambda_max_=0.04, const Real dt=1.0);
   Clik(const mRobot_min_para & mrobot_min_para_, const DiagonalMatrix & Kp_,
        const DiagonalMatrix & Ko_, const Real eps_=0.04, const Real lambda_max_=0.04,
        const Real dt=1.0);
   Clik(const Clik & x);
   ~Clik(){}
   Clik & operator=(const Clik & x);
   void q_qdot(const Quaternion & qd, const ColumnVector & pd,
               const ColumnVector & pddot, const ColumnVector & wd,
               ColumnVector & q, ColumnVector & qp);
private:
   int endeff_pos_ori_err(const ColumnVector & pd, const ColumnVector & pddot,
                          const Quaternion & qd, const ColumnVector & wd);

   Real 
     dt,                //!< Time frame.
     eps,               //!< Range of singular region in Jacobian DLS inverse.
     lambda_max;        //!< Damping factor in Jacobian DLS inverse.
   short  robot_type;   //!< Robot type used.
   Robot robot;         //!< Robot instance.
   mRobot mrobot;       //!< mRobot instance.
   mRobot_min_para mrobot_min_para; //!< mRobot_min_para instance.
   DiagonalMatrix Kp,   //!< Position error gain.
                  Ko;   //!< Orientation error gain.

   ColumnVector q ,       //!< Clik joint position.
                qp,       //!< Clik joint velocity.
                qp_prev,  //!< Clik previous joint velocity.
                Kpep,     //!< Kp times position error.
                Koe0Quat, //!< Ko times orientation error (quaternion vector part).
                v;        //!< Quaternion vector part.
};

#ifdef use_namespace
}
#endif

#endif
