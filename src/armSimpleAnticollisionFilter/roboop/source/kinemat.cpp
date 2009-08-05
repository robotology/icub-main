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
   -Rewrite mRobot::jacobian() since previous version was incorrect.
   -Added functions kine_pd().
   -Make sur that joint angles (qout) are in [-pi, pi] in inv_kin functions.

2003/05/18: Etienne Lachance
   -Added functions Robot_basic::jacobian_DLS_inv and 
    Robot/mRobot/mRobot_min_para::jacobian_dot

2003/08/22: Etienne Lachance
   -Added parameter converge in inv_kin prototype function. It indicates if the
    inverse kinematics solution converge.

2003/11/26: Etienne Lachance
   -Use angle conditions only if it converge in inv_kin.

2004/01/23: Etienne Lachance
   -Added const in non reference argument for all functions.

2004/03/12: Etienne Lachance
   -Added logic to set q in inv_kin. 

2004/04/24: Etienne Lachance
   -Moved inv_kin to invkine.cpp.

2004/05/14: Etienne Lachance
   -Replaced vec_x_prod by CrossProduct.

2004/05/21: Etienne Lachance
   -Added Doxygen comments.

2004/07/01: Ethan Tira-Thompson
    -Added support for newmat's use_namespace #define, using ROBOOP namespace

2004/07/16: Ethan Tira-Thompson
    -Supports Link::immobile flag so jacobians and deltas are 0 for immobile joints
    -Jacobians will only contain entries for mobile joints - otherwise NaNs result
     in later processing
    -Added parameters to jacobian functions to generate for frames other than 
     the end effector
-------------------------------------------------------------------------------
*/

/*!
  @file kinemat.cpp
  @brief Kinematics functions.
*/

//! @brief RCS/CVS version.
static const char rcsid[] = "$Id: kinemat.cpp,v 1.1 2007/07/24 16:03:09 amaldo Exp $";

#include "robot.h"

#ifdef use_namespace
namespace ROBOOP {
  using namespace NEWMAT;
#endif


void Robot_basic::kine(Matrix & Rot, ColumnVector & pos)const
/*!
  @brief Direct kinematics at end effector.
  @param Rot: End effector orientation.
  @param pos: Enf effector position.
*/
{
   kine(Rot,pos,dof+fix);
}

void Robot_basic::kine(Matrix & Rot, ColumnVector & pos, const int j)const
/*!
  @brief Direct kinematics at end effector.
  @param Rot: Frame j orientation.
  @param pos: Frame j position.
  @param j: Selected frame.
*/
{
   if(j < 1 || j > dof+fix) error("j must be 1 <= j <= dof+fix");

   Rot = links[1].R;
   pos = links[1].p;
   for (int i = 2; i <= j; i++) {
      pos = pos + Rot*links[i].p;
      Rot = Rot*links[i].R;
   }
}

ReturnMatrix Robot_basic::kine(void)const
//! @brief Return the end effector direct kinematics transform matrix.
{
   Matrix thomo;

   thomo = kine(dof+fix);
   thomo.Release(); return thomo;
}

ReturnMatrix Robot_basic::kine(const int j)const
//! @brief Return the frame j direct kinematics transform matrix.
{
   Matrix Rot, thomo(4,4);
   ColumnVector pos;

   kine(Rot,pos,j);
   thomo << fourbyfourident;
   thomo.SubMatrix(1,3,1,3) = Rot;
   thomo.SubMatrix(1,3,4,4) = pos;
   thomo.Release(); return thomo;
}

ReturnMatrix Robot_basic::kine_pd(const int j)const
/*!
  @brief Direct kinematics with velocity.

  Return a \f$3\times 5\f$ matrix. The first three columns
  are the frame j to the base rotation, the fourth column is
  the frame j w.r.t to the base postion vector and the last 
  column is the frame j w.r.t to the base translational 
  velocity vector. Print an error on the console if j is
  out of range.
*/
{
   Matrix temp(3,5), Rot;
   ColumnVector pos, pos_dot;

   if(j < 1 || j > dof)
     error("j must be 1 <= j <= dof");

   kine_pd(Rot, pos, pos_dot, j);

   temp.SubMatrix(1,3,1,3) = Rot;
   temp.SubMatrix(1,3,4,4) = pos;
   temp.SubMatrix(1,3,5,5) = pos_dot;
   temp.Release();
   return temp;
}

ReturnMatrix Robot_basic::jacobian_DLS_inv(const double eps, const double lambda_max,
      const int ref)const
/*!
  @brief Inverse Jacobian based on damped least squares inverse.

  @param eps: Range of singular region.
  @param lambda_max: Value to obtain a good solution in singular region.
  @param ref: Selected frame (ex: joint 4).

  The Jacobian inverse, based on damped least squares, is 
  \f[
    J^{-1}(q) = \big(J^T(q)J(q) + \lambda^2I \big)^{-1}J^T(q)
  \f]
  where \f$\lambda\f$ and \f$I\f$ is a damping factor and the identity matrix 
  respectively. Based on SVD (Singular Value Decomposition) the Jacobian is
  \f$ J = \sum_{i=1}^{m}\sigma_i u_i v_i^T\f$, where \f$u_i\f$, \f$v_i\f$ and
  \f$\sigma_i\f$ are respectively the input vector, the ouput vector and the
  singular values (\f$\sigma_1 \geq \sigma_2 \cdots \geq \sigma_r \geq 0\f$, \f$r\f$
  is the rank of J). Using the previous equations we obtain
  \f[
    J^{-1}(q) = \sum_{i=1}^{m} \frac{\sigma_i}{\sigma_i^2 + \lambda_i^2}v_iu_i
  \f]
  A singular region, based on the smallest singular value, can be defined by
  \f[
   \lambda^2 = \Bigg\{
    \begin{array}{cc}
      0 &     \textrm{si $\sigma_6 \geq \epsilon$} \\
      \Big(1 - (\frac{\sigma_6}{\epsilon})^2 \Big)\lambda^2_{max} & \textrm{sinon}
    \end{array}
  \f] 
*/
{
   Matrix jacob_inv_DLS, U, V;
   DiagonalMatrix Q;
   SVD(jacobian(ref), Q, U, V);

   if(Q(6,6) >= eps)
      jacob_inv_DLS = V*Q.i()*U.t();
   else
   {
      Q(6,6) += (1 - pow(Q(6,6)/eps,2))*lambda_max*lambda_max;
      jacob_inv_DLS = V*Q.i()*U.t();
   }

   jacob_inv_DLS.Release();
   return(jacob_inv_DLS);
}

// ---------------------  R O B O T   DH   N O T A T I O N  --------------------------

void Robot::kine_pd(Matrix & Rot, ColumnVector & pos, ColumnVector & pos_dot,
                    const int j)const
/*!
  @brief Direct kinematics with velocity.
  @param Rot: Frame j rotation matrix w.r.t to the base frame.
  @param pos: Frame j position vector wr.r.t to the base frame.
  @param pos_dot: Frame j velocity vector w.r.t to the base frame.
  @param j: Frame j.
  Print an error on the console if j is out of range.
*/
{
   if(j < 1 || j > dof)
      error("j must be 1 <= j <= dof");

   if( (pos.Nrows()!=3) || (pos.Ncols()!=1) )
       pos = ColumnVector(3); 
   if( (pos_dot.Nrows()!=3) || (pos_dot.Ncols()!=1) )
     pos_dot = ColumnVector(3);

   pos = 0.0;
   pos_dot = 0.0;
   for(int i = 1; i <= j; i++)
   {
      R[i] = R[i-1]*links[i].R;
      pos = pos + R[i-1]*links[i].p;
      pos_dot = pos_dot + CrossProduct(R[i]*w[i], R[i-1]*links[i].p);
   }
   Rot = R[j];
}

void Robot::dTdqi(Matrix & dRot, ColumnVector & dp,
                  const int i)
/*!
  @brief Partial derivative of the robot position (homogeneous transf.)

  This function computes the partial derivatives:
  \f[
  \frac{\partial{}^0 T_n}{\partial q_i} = {}^0 T_{i-1} Q_i \; {}^{i-1} T_n
  \f]
  in standard notation and
  \f[

  \frac{\partial{}^0 T_n}{\partial q_i} = {}^0 T_{i} Q_i \; {}^{i} T_n
  \f]
  in modified notation,

 with
 \f[
 Q_i = 
  \left [
    \begin{array}{cccc}
      0 & -1 & 0 & 0 \\
      1 & 0 & 0 & 0 \\
      0 & 0 & 0 & 0 \\
      0 & 0 & 0 & 0
    \end{array}
  \right ]
  \f]
 for a revolute joint and
 \f[
  Q_i = 
  \left [
   \begin{array}{cccc}
     0 & 0 & 0 & 0 \\
     0 & 0 & 0 & 0 \\
     0 & 0 & 0 & 1 \\
     0 & 0 & 0 & 0
   \end{array}
  \right ]
  \f]
  for a prismatic joint.

  \f$dRot\f$ and \f$dp\f$ are modified on output.
*/
{
   int j;
   if(i < 1 || i > dof) error("i must be 1 <= i <= dof");
   if(links[i].get_immobile()) {
      dRot = Matrix(3,3);
      dp = Matrix(3,1);
      dRot = 0.0;
      dp = 0.0;
   } else if(links[i].get_joint_type() == 0) {
      Matrix dR(3,3);
      dR = 0.0;
      Matrix R2 = links[i].R;
      ColumnVector p2 = links[i].p;
      dRot = Matrix(3,3);
      dRot << threebythreeident;
      for(j = 1; j < i; j++) {
         dRot = dRot*links[j].R;
      }
      // dRot * Q
      for(j = 1; j <= 3; j++) {
         dR(j,1) = dRot(j,2);
         dR(j,2) = -dRot(j,1);
      }
      for(j = i+1; j <= dof; j++) {
         p2 = p2 + R2*links[j].p;
         R2 = R2*links[j].R;
      }
      dp = dR*p2;
      dRot = dR*R2;
   } else {
      dRot = Matrix(3,3);
      dp = Matrix(3,1);
      dRot = 0.0;
      dp = 0.0;
      dp(3) = 1.0;
      for(j = i-1; j >= 1; j--) {
         dp = links[j].R*dp;
      }
   }
}

ReturnMatrix Robot::dTdqi(const int i)
/*!
  @brief Partial derivative of the robot position (homogeneous transf.)

  See Robot::dTdqi(Matrix & dRot, ColumnVector & dp, const int i)
  for equations.
*/
{
   Matrix dRot, thomo(4,4);
   ColumnVector dpos;

   dTdqi(dRot, dpos, i);
   thomo = (Real) 0.0;
   thomo.SubMatrix(1,3,1,3) = dRot;
   thomo.SubMatrix(1,3,4,4) = dpos;
   thomo.Release(); return thomo;
}

ReturnMatrix Robot::jacobian(const int endlink, const int ref)const
/*!
  @brief Jacobian of mobile links up to endlink expressed at frame ref.

  The Jacobian expressed in based frame is
  \f[
    ^{0}J(q) = 
    \left[ 
     \begin{array}{cccc}
      ^{0}J_1(q) & ^{0}J_2(q) & \cdots & ^{0}J_n(q) \\
     \end{array} 
   \right]
  \f]
  where \f$^{0}J_i(q)\f$ is defined by
  \f[
    ^{0}J_i(q) = 
     \begin{array}{cc}
      \left[ 
       \begin{array}{c}
        z_i \times ^{i}p_n \\
        z_i \\
       \end{array} 
      \right] & \textrm{rotoid joint} 
     \end{array} 
   \f]
   \f[
    ^{0}J_i(q) = 
      \begin{array}{cc}
       \left[ 
        \begin{array}{c}
         z_i \\
         0 \\
        \end{array} 
       \right] & \textrm{prismatic joint} \\
      \end{array}
  \f]

  Expressed in a different frame the Jacobian is obtained by
  \f[
    ^{i}J(q)  = 
    \left[
     \begin{array}{cc}
      ^{0}_iR^T & 0 \\
        0 & ^{0}_iR^T
     \end{array} 
    \right] 
   {^{0}}J(q)
  \f]
*/
{
   int i, j;
   const int adof=get_available_dof(endlink);
   Matrix jac(6,adof);
   Matrix pr, temp(3,1);

   if(ref < 0 || ref > dof) error("invalid referential");

   for(i = 1; i <= dof; i++) {
      R[i] = R[i-1]*links[i].R;
      p[i] = p[i-1]+R[i-1]*links[i].p;
   }

   for(i=1,j=1; j <= adof; i++) {
      if(links[i].get_immobile())
         continue;
      if(links[i].get_joint_type() == 0) {
         temp(1,1) = R[i-1](1,3);
         temp(2,1) = R[i-1](2,3);
         temp(3,1) = R[i-1](3,3);
         pr = p[dof]-p[i-1];
         temp = CrossProduct(temp,pr);
         jac(1,j) = temp(1,1);
         jac(2,j) = temp(2,1);
         jac(3,j) = temp(3,1);
         jac(4,j) = R[i-1](1,3);
         jac(5,j) = R[i-1](2,3);
         jac(6,j) = R[i-1](3,3);
      } else {
         jac(1,j) = R[i-1](1,3);
         jac(2,j) = R[i-1](2,3);
         jac(3,j) = R[i-1](3,3);
         jac(4,j) = jac(5,j) = jac(6,j) = 0.0;
      }
      j++;
   }

   if(ref != 0) {
      Matrix zeros(3,3);
      zeros = (Real) 0.0;
      Matrix RT = R[ref].t();
      Matrix Rot;
      Rot = ((RT & zeros) | (zeros & RT));
      jac = Rot*jac;
   }
   jac.Release(); return jac;
}

ReturnMatrix Robot::jacobian_dot(const int ref)const
/*!
  @brief Jacobian derivative of mobile joints expressed at frame ref.

  The Jacobian derivative expressed in based frame is
  \f[
   ^{0}\dot{J}(q,\dot{q}) = 
    \left[ 
     \begin{array}{cccc}
       ^{0}\dot{J}_1(q,\dot{q}) & ^{0}\dot{J}_2(q,\dot{q}) & \cdots & ^{0}\dot{J}_n(q,\dot{q}) \\
     \end{array} 
    \right]
  \f]
  where \f$^{0}\dot{J}_i(q,\dot{q})\f$ is defined by
  \f[
   ^{0}\dot{J}_i(q,\dot{q}) = 
    \begin{array}{cc}
     \left[ 
      \begin{array}{c}
       \omega_{i-1} \times z_i \\
       \omega_{i-1} \times ^{i-1}p_n + z_i \times ^{i-1}\dot{p}_n
     \end{array} 
    \right] & \textrm{rotoid joint} 
   \end{array}
  \f]
  \f[
   ^{0}\dot{J}_i(q,\dot{q}) =
   \begin{array}{cc}
    \left[ 
     \begin{array}{c}
       0 \\
       0 \\
     \end{array} 
    \right] & \textrm{prismatic joint} \\
   \end{array}
  \f]
  Expressed in a different frame the Jacobian derivative is obtained by
  \f[
    ^{i}J(q)  = 
    \left[
     \begin{array}{cc}
      ^{0}_iR^T & 0 \\
        0 & ^{0}_iR^T
     \end{array} 
    \right] 
   {^{0}}J(q)
  \f]
*/
{
   int i, j;
   const int adof=get_available_dof();
   Matrix jacdot(6,adof);
   ColumnVector e(3), temp, pr, ppr;

   if(ref < 0 || ref > dof)
      error("invalid referential");

   for(i = 1; i <= dof; i++)
   {
      R[i] = R[i-1]*links[i].R;
      p[i] = p[i-1] + R[i-1]*links[i].p;
      pp[i] = pp[i-1] + CrossProduct(R[i]*w[i], R[i-1]*links[i].p);
   }

   for(i=1,j=1; j <= adof; i++) {
      if(links[i].get_immobile())
				 continue;
      if(links[i].get_joint_type() == 0)
      {
         pr = p[dof]-p[i-1];
         ppr = pp[dof]-pp[i-1];
         e(1) = R[i-1](1,3);
         e(2) = R[i-1](2,3);
         e(3) = R[i-1](3,3);
         temp = CrossProduct(R[i-1]*w[i-1], e);
         jacdot(4,j) = temp(1);           // d(e)/dt
         jacdot(5,j) = temp(2);
         jacdot(6,j) = temp(3);
         temp = CrossProduct(temp,pr) + CrossProduct(e,ppr);
         jacdot(1,j) = temp(1);
         jacdot(2,j) = temp(2);
         jacdot(3,j) = temp(3);
      }
      else
         jacdot(1,j) = jacdot(2,j) = jacdot(3,j) =
            jacdot(4,j) = jacdot(5,j) = jacdot(6,j) = 0.0;
      j++;
   }

   if(ref != 0) {
      Matrix zeros(3,3);
      zeros = (Real) 0.0;
      Matrix RT = R[ref].t();
      Matrix Rot;
      Rot = ((RT & zeros) | (zeros & RT));
      jacdot = Rot*jacdot;
   }

   jacdot.Release(); return jacdot;
}

// ----------------  R O B O T   M O D I F I E D   DH   N O T A T I O N  --------------------------

void mRobot::kine_pd(Matrix & Rot, ColumnVector & pos, ColumnVector & pos_dot,
                     const int j)const
/*!
  @brief Direct kinematics with velocity.
  @param Rot: Frame j rotation matrix w.r.t to the base frame.
  @param pos: Frame j position vector wr.r.t to the base frame.
  @param pos_dot: Frame j velocity vector w.r.t to the base frame.
  @param j: Frame j.
  Print an error on the console if j is out of range.
*/
{
   if(j < 1 || j > dof+fix)
      error("j must be 1 <= j <= dof+fix");

   if( (pos.Nrows()!=3) || (pos.Ncols()!=1) )
       pos = ColumnVector(3); 
   if( (pos_dot.Nrows()!=3) || (pos_dot.Ncols()!=1) )
     pos_dot = ColumnVector(3);

   pos = 0.0;
   pos_dot = 0.0;
   for(int i = 1; i <= j; i++)
   {
      pos = pos + R[i-1]*links[i].p;
      pos_dot = pos_dot + R[i-1]*CrossProduct(w[i-1], links[i].p);
      R[i] = R[i-1]*links[i].R;
   }
   Rot = R[j];
}

void mRobot::dTdqi(Matrix & dRot, ColumnVector & dp, const int i)
/*!
  @brief Partial derivative of the robot position (homogeneous transf.)

  This function computes the partial derivatives:
  \f[
  \frac{\partial{}^0 T_n}{\partial q_i} = {}^0 T_{i} Q_i \; {}^{i} T_n
  \f]
  with
  \f[
  Q_i = 
  \left [
    \begin{array}{cccc}
      0 & -1 & 0 & 0 \\
      1 & 0 & 0 & 0 \\
      0 & 0 & 0 & 0 \\
      0 & 0 & 0 & 0
    \end{array}
  \right ]
  \f]
 for a revolute joint and
 \f[
  Q_i = 
  \left [
   \begin{array}{cccc}
     0 & 0 & 0 & 0 \\
     0 & 0 & 0 & 0 \\
     0 & 0 & 0 & 1 \\
     0 & 0 & 0 & 0
   \end{array}
  \right ]
  \f]
  for a prismatic joint.

  \f$dRot\f$ and \f$dp\f$ are modified on output.
*/
{
   int j;
   if(i < 1 || i > dof) error("i must be 1 <= i <= dof");
   if(links[i].get_immobile()) {
      dRot = Matrix(3,3);
      dp = Matrix(3,1);
      dRot = 0.0;
      dp = 0.0;
   } else if(links[i].get_joint_type() == 0) {
      Matrix dR(3,3), R2(3,3), p2(3,1);
      dR = 0.0;
      dRot = Matrix(3,3);
      dRot << threebythreeident;
      for(j = 1; j <= i; j++) {
         dRot = dRot*links[j].R;
      }
      // dRot * Q
      for(j = 1; j <= 3; j++) {
         dR(j,1) = dRot(j,2);
         dR(j,2) = -dRot(j,1);
      }
      if(i < dof) {
         R2 = links[i+1].R;
         p2 = links[i+1].p;
      } else {
         R2 <<  threebythreeident;
         p2 = 0.0;
      }
      for(j = i+1; j <= dof; j++) {
         p2 = p2 + R2*links[j].p;
         R2 = R2*links[j].R;
      }
      dp = dR*p2;
      dRot = dR*R2;  // probleme ...
   } else {
      dRot = Matrix(3,3);
      dp = Matrix(3,1);
      dRot = 0.0;
      dp = 0.0;
      dp(3) = 1.0;
      for(j = i; j >= 1; j--) {
         dp = links[j].R*dp;
      }
   }
}

ReturnMatrix mRobot::dTdqi(const int i)
/*!
  @brief Partial derivative of the robot position (homogeneous transf.)

  See mRobot::dTdqi(Matrix & dRot, ColumnVector & dp, const int i)
  for equations.
*/
{
   Matrix dRot, thomo(4,4);
   ColumnVector dpos;

   dTdqi(dRot, dpos, i);
   thomo = (Real) 0.0;
   thomo.SubMatrix(1,3,1,3) = dRot;
   thomo.SubMatrix(1,3,4,4) = dpos;
   thomo.Release(); return thomo;
}

ReturnMatrix mRobot::jacobian(const int endlink, const int ref)const
/*!
  @brief Jacobian of mobile joints up to endlink expressed at frame ref.

  See Robot::jacobian for equations.
*/
{
   int i, j;
   const int adof=get_available_dof(endlink);
   Matrix jac(6,adof);
   ColumnVector pr(3), temp(3);

   if(ref < 0 || ref > dof+fix)
      error("invalid referential");

   for(i = 1; i <= dof+fix; i++) {
      R[i] = R[i-1]*links[i].R;
      p[i] = p[i-1] + R[i-1]*links[i].p;
   }

   for(i=1,j=1; j <= adof; i++) {
      if(links[i].get_immobile())
         continue;
      if(links[i].get_joint_type() == 0){
         temp(1) = R[i](1,3);
         temp(2) = R[i](2,3);
         temp(3) = R[i](3,3);
         pr = p[dof+fix]-p[i];
         temp = CrossProduct(temp,pr);
         jac(1,j) = temp(1);
         jac(2,j) = temp(2);
         jac(3,j) = temp(3);
         jac(4,j) = R[i](1,3);
         jac(5,j) = R[i](2,3);
         jac(6,j) = R[i](3,3);
      } else {
         jac(1,j) = R[i](1,3);
         jac(2,j) = R[i](2,3);
         jac(3,j) = R[i](3,3);
         jac(4,j) = jac(5,j) = jac(6,j) = 0.0;
      }
      j++;
   }
   if(ref != 0) {
      Matrix zeros(3,3);
      zeros = (Real) 0.0;
      Matrix RT = R[ref].t();
      Matrix Rot;
      Rot = ((RT & zeros) | (zeros & RT));
      jac = Rot*jac;
   }
   jac.Release(); return jac;
}

ReturnMatrix mRobot::jacobian_dot(const int ref)const
/*!
  @brief Jacobian derivative of mobile joints expressed at frame ref.

  See Robot::jacobian_dot for equations.
*/
{
   int i, j;
   const int adof=get_available_dof();
   Matrix jacdot(6,adof);
   ColumnVector e(3), temp, pr, ppr;

   if(ref < 0 || ref > dof+fix)
      error("invalid referential");

   for(i = 1; i <= dof+fix; i++)
   {
      R[i] = R[i-1]*links[i].R;
      p[i] = p[i-1] + R[i-1]*links[i].p;
      pp[i] = pp[i-1] + R[i-1]*CrossProduct(w[i-1], links[i].p);
   }

   for(i=1,j=1; j <= adof; i++) {
      if(links[i].get_immobile())
         continue;
      if(links[i].get_joint_type() == 0)
      {
         pr = p[dof+fix]-p[i];
         ppr = pp[dof+fix]-pp[i];

         e(1) = R[i](1,3);
         e(2) = R[i](2,3);
         e(3) = R[i](3,3);
         temp = CrossProduct(R[i-1]*w[i-1], e);
         jacdot(4,j) = temp(1);           // d(e)/dt
         jacdot(5,j) = temp(2);
         jacdot(6,j) = temp(3);

         temp = CrossProduct(temp,pr) + CrossProduct(e,ppr);
         jacdot(1,j) = temp(1);
         jacdot(2,j) = temp(2);
         jacdot(3,j) = temp(3);
      }
      else
         jacdot(1,j) = jacdot(2,j) = jacdot(3,j) =
            jacdot(4,j) = jacdot(5,j) = jacdot(6,j) = 0.0;
      j++;
   }

   if(ref != 0) {
      Matrix zeros(3,3);
      zeros = (Real) 0.0;
      Matrix RT = R[ref].t();
      Matrix Rot;
      Rot = ((RT & zeros) | (zeros & RT));
      jacdot = Rot*jacdot;
   }

   jacdot.Release(); return jacdot;
}

// ------------- R O B O T  DH  M O D I F I E D,  M I N I M U M   P A R A M E T E R S  ------------

void mRobot_min_para::kine_pd(Matrix & Rot, ColumnVector & pos, ColumnVector & pos_dot,
                              const int j)const
/*!
  @brief Direct kinematics with velocity.
  @param Rot: Frame j rotation matrix w.r.t to the base frame.
  @param pos: Frame j position vector wr.r.t to the base frame.
  @param pos_dot: Frame j velocity vector w.r.t to the base frame.
  @param j: Frame j.
  Print an error on the console if j is out of range.
*/
{
   if(j < 1 || j > dof+fix)
      error("j must be 1 <= j <= dof+fix");

   if( (pos.Nrows()!=3) || (pos.Ncols()!=1) )
       pos = ColumnVector(3); 
   if( (pos_dot.Nrows()!=3) || (pos_dot.Ncols()!=1) )
     pos_dot = ColumnVector(3);

   pos = 0.0;
   pos_dot = 0.0;
   for(int i = 1; i <= j; i++)
   {
      pos = pos + R[i-1]*links[i].p;
      pos_dot = pos_dot + R[i-1]*CrossProduct(w[i-1], links[i].p);
      R[i] = R[i-1]*links[i].R;
   }
   Rot = R[j];
}

void mRobot_min_para::dTdqi(Matrix & dRot, ColumnVector & dp, const int i)
/*!
  @brief Partial derivative of the robot position (homogeneous transf.)

  This function computes the partial derivatives:
  \f[
  \frac{\partial{}^0 T_n}{\partial q_i} = {}^0 T_{i} Q_i \; {}^{i} T_n
  \f]

  See mRobot::dTdqi(Matrix & dRot, ColumnVector & dp, const int i)
  for equations.
*/
{
   int j;
   if(i < 1 || i > dof) error("i must be 1 <= i <= dof");
   if(links[i].get_immobile()) {
      dRot = Matrix(3,3);
      dp = Matrix(3,1);
      dRot = 0.0;
      dp = 0.0;
   } else if(links[i].get_joint_type() == 0) {
      Matrix dR(3,3), R2, p2(3,1);
      dR = 0.0;
      dRot = Matrix(3,3);
      dRot << threebythreeident;
      for(j = 1; j <= i; j++) {
         dRot = dRot*links[j].R;
      }
      // dRot * Q
      for(j = 1; j <= 3; j++) {
         dR(j,1) = dRot(j,2);
         dR(j,2) = -dRot(j,1);
      }
      if(i < dof) {
         R2 = links[i+1].R;
         p2 = links[i+1].p;
      } else {
         R2 <<  threebythreeident;
         p2 = 0.0;
      }
      for(j = i+1; j <= dof; j++) {
         p2 = p2 + R2*links[j].p;
         R2 = R2*links[j].R;
      }
      dp = dR*p2;
      dRot = dR*R2;
   } else {
      dRot = Matrix(3,3);
      dp = Matrix(3,1);
      dRot = 0.0;
      dp = 0.0;
      dp(3) = 1.0;
      for(j = i; j >= 1; j--) {
         dp = links[j].R*dp;
      }
   }
}

ReturnMatrix mRobot_min_para::dTdqi(const int i)
/*!
  @brief Partial derivative of the robot position (homogeneous transf.)

  See mRobot::dTdqi(Matrix & dRot, ColumnVector & dp, const int i)
  for equations.
*/
{
   Matrix dRot, thomo(4,4);
   ColumnVector dpos;

   dTdqi(dRot, dpos, i);
   thomo = (Real) 0.0;
   thomo.SubMatrix(1,3,1,3) = dRot;
   thomo.SubMatrix(1,3,4,4) = dpos;
   thomo.Release(); return thomo;
}

ReturnMatrix mRobot_min_para::jacobian(const int endlink, const int ref)const
/*!
  @brief Jacobian of mobile joints up to endlink expressed at frame ref.

  See Robot::jacobian for equations.
*/
{
   int i, j;
   const int adof=get_available_dof(endlink);
   Matrix jac(6,adof);
   ColumnVector pr(3), temp(3);

   if(ref < 0 || ref > dof+fix)
      error("invalid referential");

   for(i = 1; i <= dof+fix; i++) {
      R[i] = R[i-1]*links[i].R;
      p[i] = p[i-1] + R[i-1]*links[i].p;
   }

   for(i=1,j=1; j<=adof; i++) {
      if(links[i].get_immobile())
         continue;
      if(links[i].get_joint_type() == 0){
         temp(1) = R[i](1,3);
         temp(2) = R[i](2,3);
         temp(3) = R[i](3,3);

         pr = p[dof+fix]-p[i];
         temp = CrossProduct(temp,pr);
         jac(1,j) = temp(1);
         jac(2,j) = temp(2);
         jac(3,j) = temp(3);
         jac(4,j) = R[i](1,3);
         jac(5,j) = R[i](2,3);
         jac(6,j) = R[i](3,3);
      } else {
         jac(1,j) = R[i](1,3);
         jac(2,j) = R[i](2,3);
         jac(3,j) = R[i](3,3);
         jac(4,j) = jac(5,j) = jac(6,j) = 0.0;
      }
      j++;
   }
   if(ref != 0) {
      Matrix zeros(3,3);
      zeros = (Real) 0.0;
      Matrix RT = R[ref].t();
      Matrix Rot;
      Rot = ((RT & zeros) | (zeros & RT));
      jac = Rot*jac;
   }

   jac.Release(); return jac;
}

ReturnMatrix mRobot_min_para::jacobian_dot(const int ref)const
/*!
  @brief Jacobian derivative of mobile joints expressed at frame ref.

  See Robot::jacobian_dot for equations.
*/
{
   int i, j;
   const int adof=get_available_dof();
   Matrix jacdot(6,adof);
   ColumnVector e(3), temp, pr, ppr;

   if(ref < 0 || ref > dof+fix)
      error("invalid referential");

   for(i = 1; i <= dof+fix; i++)
   {
      R[i] = R[i-1]*links[i].R;
      p[i] = p[i-1] + R[i-1]*links[i].p;
      pp[i] = pp[i-1] + R[i-1]*CrossProduct(w[i-1], links[i].p);
   }

   for(i=1,j=1; j <= dof; i++) {
      if(links[i].get_immobile())
         continue;
      if(links[i].get_joint_type() == 0)
      {
         pr = p[dof+fix]-p[i];
         ppr = pp[dof+fix]-pp[i];

         e(1) = R[i](1,3);
         e(2) = R[i](2,3);
         e(3) = R[i](3,3);
         temp = CrossProduct(R[i-1]*w[i-1], e);
         jacdot(4,j) = temp(1);           // d(e)/dt
         jacdot(5,j) = temp(2);
         jacdot(6,j) = temp(3);

         temp = CrossProduct(temp,pr) + CrossProduct(e,ppr);
         jacdot(1,j) = temp(1);
         jacdot(2,j) = temp(2);
         jacdot(3,j) = temp(3);
      }
      else
         jacdot(1,j) = jacdot(2,j) = jacdot(3,j) =
            jacdot(4,j) = jacdot(5,j) = jacdot(6,j) = 0.0;
      j++;
   }

   if(ref != 0) {
      Matrix zeros(3,3);
      zeros = (Real) 0.0;
      Matrix RT = R[ref].t();
      Matrix Rot;
      Rot = ((RT & zeros) | (zeros & RT));
      jacdot = Rot*jacdot;
   }

   jacdot.Release(); return jacdot;
}

#ifdef use_namespace
}
#endif
