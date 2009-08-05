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

2004/07/01: Etienne Lachance
   -Added doxygen documentation.

2004/07/01: Ethan Tira-Thompson
    -Added support for newmat's use_namespace #define, using ROBOOP namespace.
*/

//! @brief RCS/CVS version.
static const char rcsid[] = "$Id: clik.cpp,v 1.1 2007/07/24 16:03:09 amaldo Exp $";

/*!
  @file clik.cpp
  @brief Clik member functions.
*/

#include "quaternion.h"
#include "clik.h"

#ifdef use_namespace
namespace ROBOOP {
  using namespace NEWMAT;
#endif

/*!
  @fn Clik::Clik(const Robot & robot_, const DiagonalMatrix & Kp_, const DiagonalMatrix & Ko_,
                 const Real eps_, const Real lambda_max_, const Real dt);
  @brief Constructor.
*/
Clik::Clik(const Robot & robot_, const DiagonalMatrix & Kp_, const DiagonalMatrix & Ko_,
           const Real eps_, const Real lambda_max_, const Real dt_):
      dt(dt_),
      eps(eps_),
      lambda_max(lambda_max_),
      robot(robot_)
{
   robot_type = CLICK_DH;
   // Initialize with same joint position (and rates) has the robot.
   q = robot.get_q();
   qp = robot.get_qp();
   qp_prev = qp;
   Kpep = ColumnVector(3); Kpep = 0;
   Koe0Quat = ColumnVector(3); Koe0Quat = 0;
   v = ColumnVector(6); v = 0;

   if(Kp_.Nrows()==3)
      Kp = Kp_;
   else
   {
      Kp = DiagonalMatrix(3); Kp = 0.0;
      cerr << "Clik::Clik-->Robot, Kp if not 3x3, set gain to 0." << endl;
   }
   if(Ko_.Nrows()==3)
      Ko = Ko_;
   else
   {
      Ko = DiagonalMatrix(3); Ko = 0.0;
      cerr << "Clik::Clik-->Robot, Ko if not 3x3, set gain to 0." << endl;
   }
}


/*!
  @fn Clik::Clik(const mRobot & mrobot_, const DiagonalMatrix & Kp_, const DiagonalMatrix & Ko_,
                 const Real eps_, const Real lambda_max_, const Real dt);
  @brief Constructor.
*/
Clik::Clik(const mRobot & mrobot_, const DiagonalMatrix & Kp_, const DiagonalMatrix & Ko_,
           const Real eps_, const Real lambda_max_, const Real dt_):
      dt(dt_),
      eps(eps_),
      lambda_max(lambda_max_),
      mrobot(mrobot_)
{
   robot_type = CLICK_mDH;
   // Initialize with same joint position (and rates) has the robot.
   q = mrobot.get_q();
   qp = mrobot.get_qp();
   qp_prev = qp;
   Kpep = ColumnVector(3); Kpep = 0;
   Koe0Quat = ColumnVector(3); Koe0Quat = 0;
   v = ColumnVector(6); v = 0;

   if(Kp_.Nrows()==3)
      Kp = Kp_;
   else
   {
      Kp = DiagonalMatrix(3); Kp = 0.0;
      cerr << "Clik::Clik-->mRobot, Kp if not 3x3, set gain to 0." << endl;
   }
   if(Ko_.Nrows()==3)
      Ko = Ko_;
   else
   {
      Ko = DiagonalMatrix(3); Ko = 0.0;
      cerr << "Clik::Cli, Ko if not 3x3, set gain to 0." << endl;
   }
}


/*!
  @fn Clik::Clik(const mRobot_min_para & mrobot_min_para_, const DiagonalMatrix & Kp_, const DiagonalMatrix & Ko_,
                 const Real eps_, const Real lambda_max_, const Real dt);
  @brief Constructor.
*/
Clik::Clik(const mRobot_min_para & mrobot_min_para_, const DiagonalMatrix & Kp_,
           const DiagonalMatrix & Ko_, const Real eps_, const Real lambda_max_,
           const Real dt_):
      dt(dt_),
      eps(eps_),
      lambda_max(lambda_max_),
      mrobot_min_para(mrobot_min_para_)
{
   robot_type = CLICK_mDH_min_para;
   // Initialize with same joint position (and rates) has the robot.
   q = mrobot_min_para.get_q();
   qp = mrobot_min_para.get_qp();
   qp_prev = qp;
   Kpep = ColumnVector(3); Kpep = 0;
   Koe0Quat = ColumnVector(3); Koe0Quat = 0;
   v = ColumnVector(6); v = 0;

   if(Kp_.Nrows()==3)
      Kp = Kp_;
   else
   {
      Kp = DiagonalMatrix(3); Kp = 0.0;
      cerr << "Clik::Clik-->mRobot, Kp if not 3x3, set gain to 0." << endl;
   }
   if(Ko_.Nrows()==3)
      Ko = Ko_;
   else
   {
      Ko = DiagonalMatrix(3); Ko = 0.0;
      cerr << "Clik::Cli, Ko if not 3x3, set gain to 0." << endl;
   }
}


Clik::Clik(const Clik & x)
//!  @brief Copy constructor.
{
   robot_type = x.robot_type;
   switch(robot_type)
   {
   case CLICK_DH:
      robot = x.robot;
      break;
   case CLICK_mDH:
      mrobot = x.mrobot;
      break;
   case CLICK_mDH_min_para:
      mrobot_min_para = x.mrobot_min_para;
      break;
   }
   eps = x.eps;
   lambda_max = x.lambda_max;
   dt = x.dt;
   q = x.q;
   qp = x.qp;
   qp_prev = x.qp_prev;
   Kpep = x.Kpep;
   Koe0Quat = x.Koe0Quat;
   Kp = x.Kp;
   Ko = x.Ko;
   v = x.v;
}

Clik & Clik::operator=(const Clik & x)
//!  @brief Overload = operator.
{
   robot_type = x.robot_type;
   switch(robot_type)
   {
   case CLICK_DH:
      robot = x.robot;
      break;
   case CLICK_mDH:
      mrobot = x.mrobot;
      break;
   case CLICK_mDH_min_para:
      mrobot_min_para = x.mrobot_min_para;
      break;
   }
   eps = x.eps;
   lambda_max = x.lambda_max;
   dt = x.dt;
   q = x.q;
   qp = x.qp;
   qp_prev = x.qp_prev;
   Kpep = x.Kpep;
   Koe0Quat = x.Koe0Quat;
   Kp = x.Kp;
   Ko = x.Ko;
   v = x.v;

   return *this;
}


int Clik::endeff_pos_ori_err(const ColumnVector & pd, const ColumnVector & pdd,
                             const Quaternion & qqqd, const ColumnVector & wd)
/*!
  @brief Obtain end effector position and orientation error.
  @param pd: Desired eff position in base frame.
  @param pdd: Desired eff velocity in base frame.
  @param qqqd: Desired eff orientation in base frame.
  @param wd: Desired eff angular velocity in base frame.
*/
{
   ColumnVector p;
   Matrix R;

   switch(robot_type)
   {
   case CLICK_DH:
      robot.set_q(q);
      robot.kine(R, p);  // In base frame
      break;
   case CLICK_mDH:
      mrobot.set_q(q);
      mrobot.kine(R, p);
      break;
   case CLICK_mDH_min_para:
      mrobot_min_para.set_q(q);
      mrobot_min_para.kine(R, p);
      break;
   }
   Kpep = Kp*(pd - p);
   Quaternion qq(R);

   Quaternion qqd;

   if(qq.dot_prod(qqqd) < 0)
      qqd = qqqd*(-1);
   else
      qqd = qqqd;

   // quaternion error on vectoriel part. We used equation 42 [4] instead equation 23 [1].
   Koe0Quat = Ko*(qq.s()*qqd.v() - qqd.s()*qq.v() + x_prod_matrix(qq.v())*qqd.v());

   return 0;
}


void Clik::q_qdot(const Quaternion & qd, const ColumnVector & pd,
                  const ColumnVector & pdd, const ColumnVector & wd,
                  ColumnVector & q_, ColumnVector & qp_)
/*!
  @brief Obtain joints position and velocity.
  @param qd: Desired eff orientatio in base frame.
  @param pd: Desired eff position in base frame.
  @param pdd: Desired eff velocity in base frame.
  @param wd: Desired eff angular velocity in base frame.
  @param q_: Output joint position.
  @param qp_: Output joint velocity.
*/
{
   v.SubMatrix(1,3,1,1) = pdd + Kpep;
   v.SubMatrix(4,6,1,1) = wd + Koe0Quat;

   switch(robot_type)
   {
   case CLICK_DH:
      robot.set_q(q);
      qp = robot.jacobian_DLS_inv(eps, lambda_max)*v;
      break;
   case CLICK_mDH:
      mrobot.set_q(q);
      qp = mrobot.jacobian_DLS_inv(eps, lambda_max)*v;
      break;
   case CLICK_mDH_min_para:
      mrobot_min_para.set_q(q);
      qp = mrobot_min_para.jacobian_DLS_inv(eps, lambda_max)*v;
      break;
   }

   q = q + Integ_Trap(qp, qp_prev, dt);
   endeff_pos_ori_err(pd, pdd, qd, wd);

   q_ = q;
   qp_ = qp;
}

#ifdef use_namespace
}
#endif
