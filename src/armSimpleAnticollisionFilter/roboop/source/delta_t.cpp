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

2003/29/04: Etienne Lachance
   -Fix Robot::delta_torque.
   -Added mRobot/mRobot_min_para::delta_torque.

2004/05/14: Etienne Lachance
   -Replaced vec_x_prod by CrossProduct.

2004/07/01: Ethan Tira-Thompson
    -Added support for newmat's use_namespace #define, using ROBOOP namespace

2004/07/02: Etienne Lachance
   -Added Doxygen comments.
-------------------------------------------------------------------------------
*/

/*!
  @file delta_t.cpp
  @brief Delta torque (linearized dynamics).
*/

//! @brief RCS/CVS version.
static const char rcsid[] = "$Id: delta_t.cpp,v 1.1 2007/07/24 16:03:09 amaldo Exp $";

#include "robot.h"

#ifdef use_namespace
namespace ROBOOP {
  using namespace NEWMAT;
#endif


void Robot::delta_torque(const ColumnVector & q, const ColumnVector & qp,
                         const ColumnVector & qpp, const ColumnVector & dq,
                         const ColumnVector & dqp, const ColumnVector & dqpp,
                         ColumnVector & ltorque, ColumnVector & dtorque)
/*!
  @brief Delta torque dynamics.

  This function computes
  \f[
    \delta \tau = D(q) \delta \ddot{q}
    + S_1(q,\dot{q}) \delta \dot{q} + S_2(q,\dot{q},\ddot{q}) \delta q    
  \f]

  Murray and Neuman Cite_: Murray86 have developed an efficient recursive 
  linearized Newton-Euler formulation.   In order to apply the RNE as presented in 
  let us define the following variables 

  \f[
    p_{di} = \frac{\partial p_i}{\partial d_i} = 
    \left [
      \begin{array}{ccc}
        0 & \sin \alpha_i & \cos \alpha_i 
      \end{array}
    \right ]^T
  \f]
  \f[
    Q = 
    \left [
      \begin{array}{ccc}
        0 & -1 & 0  \\
	1 & 0 & 0  \\
	0 & 0 & 0 
      \end{array}
  \right ]
  \f]

  Forward Iterations for \f$i=1, 2, \ldots, n\f$. 
  Initialize: \f$\delta \omega_0 = \delta \dot{\omega}_0 = \delta \dot{v}_0 = 0\f$.
  \f[
    \delta \omega_i = R_i^T \{\delta \omega_{i-1} + \sigma_i [ z_0 \delta \dot{\theta}_i 
    - Q(\omega_{i-1} + \dot{\theta}_i ) \delta \theta_i ] \} 
  \f]
  \f[
  \delta \dot{\omega}_i = R_i^T  \{ \delta \dot{\omega}_{i-1} + 
  \sigma_i [z_0 \delta \ddot{\theta}_i + \delta \omega_{i-1} \times (z_0 \dot{\theta}_i )
  + \omega_{i-1} \times (z_0 \delta \dot{\theta}_i )] 
  - \sigma_i Q [ \omega_{i-1} + z_0 \ddot{\theta}_i 
  + \omega_{i-1} \times (z_0 \dot{\theta}_i )] \delta \theta_i \}
  \f]

  \f[
   \delta \dot{v}_i = R_i^T  \{ \delta \dot{v}_{i-1} - \sigma_i Q \dot{v}_{i-1} \delta \theta_i
   + (1 -\sigma_i) [z_0 \delta \ddot{d}_i + 2 \delta \omega_{i-1} \times (z_0 \dot{d}_i ) 
   + 2 \omega_{i-1} \times (z_0 \delta \dot{d}_i )] \} + \delta \dot{\omega}_i \times p_i 
   + \delta \omega_i \times ( \omega_i \times p_i) + \omega_i \times ( \delta \omega_i \times p_i)
   + (1 - \sigma_i) (\dot{\omega}_i \times p_{di} 
   + \omega_i \times ( \omega_i \times p_{di}) ) \delta d_i
  \f]

  Backward Iterations for \f$i=n, n-1, \ldots, 1\f$. 
  Initialize: \f$\delta f_{n+1} = \delta n_{n+1} = 0\f$.

  \f[
  \delta \dot{v}_{ci} =
  \delta v_i + \delta \omega_i \times r_i 
  + \delta \omega_i \times (\omega_i \times r_i) 
  + \omega_i \times (\delta \omega_i \times r_i) 
  \f]
  \f[
  \delta F_i = m_i \delta \dot{v}_{ci}
  \f]
  \f[
  \delta N_i = I_{ci} \delta \dot{\omega}_i 
  + \delta \omega_i \times (I_{ci} \omega_i) 
  + \omega_i \times (I_{ci} \delta \omega_i) 
  \f]
  \f[
  \delta f_i = R_{i+1} [ \delta f_{i+1} ]  
  + \delta F_{i} + \sigma_{i+1} Q R_{i+1} [ f_{i+1} ] \delta \theta_{i+1}
  \f]
  \f[
  \delta n_i = R_{i+1} [ \delta n_{i+1} ]  
  + \delta N_{i} + p_{i} \times \delta f_{i} 
  + r_{i} \times \delta F_{i} + (1 - \sigma_i) (p_{di} \times f_{i}) \delta d_i 
  + \sigma_{i+1} Q R_{i+1} [ n_{i+1} ] \delta \theta_{i+1}
  \f]
  \f[
  \delta \tau_i = \sigma_i [ \delta n_i^T (R_i^T z_0) 
  - n_i^T (R_i^T Q z_0) \delta \theta_i] + (1 -\sigma_i) [ \delta f_i^T (R_i^T z_0) ]
  \f]
*/
{
   int i;
   Matrix Rt, temp;
   if(q.Ncols() != 1 || q.Nrows() != dof) error("q has wrong dimension");
   if(qp.Ncols() != 1 || qp.Nrows() != dof) error("qp has wrong dimension");
   if(qpp.Ncols() != 1 || qpp.Nrows() != dof) error("qpp has wrong dimension");
   if(dq.Ncols() != 1 || dq.Nrows() != dof) error("dq has wrong dimension");
   if(dqp.Ncols() != 1 || dqp.Nrows() != dof) error("dqp has wrong dimension");
   if(dqpp.Ncols() != 1 || dqpp.Nrows() != dof) error("dqpp has wrong dimension");
   ltorque = ColumnVector(dof);
   dtorque = ColumnVector(dof);
   set_q(q);

   vp[0] = gravity;
   ColumnVector z0(3);
   z0(1) = 0.0; z0(2) = 0.0;  z0(3) = 1.0;
   Matrix Q(3,3);
   Q = 0.0;
   Q(1,2) = -1.0;
   Q(2,1) = 1.0;
   for(i = 1; i <= dof; i++)
   {
      Rt = links[i].R.t();
      p[i] = ColumnVector(3);
      p[i](1) = links[i].get_a();
      p[i](2) = links[i].get_d() * Rt(2,3);
      p[i](3) = links[i].get_d() * Rt(3,3);
      if(links[i].get_joint_type() != 0)
      {
         dp[i] = ColumnVector(3);
         dp[i](1) = 0.0;
         dp[i](2) = Rt(2,3);
         dp[i](3) = Rt(3,3);
      }
      if(links[i].get_joint_type() == 0)
      {
         w[i] = Rt*(w[i-1] + z0*qp(i));
         dw[i] = Rt*(dw[i-1] + z0*dqp(i)
                     - Q*(w[i-1] + z0*qp(i))*dq(i));
         wp[i] = Rt*(wp[i-1] + z0*qpp(i)
                     + CrossProduct(w[i-1],z0*qp(i)));
         dwp[i] = Rt*(dwp[i-1] + z0*dqpp(i)
                      + CrossProduct(dw[i-1],z0*qp(i))
                      + CrossProduct(w[i-1],z0*dqp(i))
                      - Q*(wp[i-1]+z0*qpp(i)+CrossProduct(w[i-1],z0*qp(i)))
                      *dq(i));
         vp[i] = CrossProduct(wp[i],p[i])
                 + CrossProduct(w[i],CrossProduct(w[i],p[i]))
                 + Rt*(vp[i-1]);
         dvp[i] = CrossProduct(dwp[i],p[i])
                  + CrossProduct(dw[i],CrossProduct(w[i],p[i]))
                  + CrossProduct(w[i],CrossProduct(dw[i],p[i]))
                  + Rt*(dvp[i-1] - Q*vp[i-1]*dq(i));
      }
      else
      {
         w[i] = Rt*w[i-1];
         dw[i] = Rt*dw[i-1];
         wp[i] = Rt*wp[i-1];
         dwp[i] = Rt*dwp[i-1];
         vp[i] = Rt*(vp[i-1] + z0*qpp(i)
                     + 2.0*CrossProduct(w[i-1],z0*qp(i)))
                 + CrossProduct(wp[i],p[i])
                 + CrossProduct(w[i],CrossProduct(w[i],p[i]));
         dvp[i] = Rt*(dvp[i-1] + z0*dqpp(i)
                      + 2.0*(CrossProduct(dw[i-1],z0*qp(i))
                             + CrossProduct(w[i-1],z0*dqp(i))))
                  + CrossProduct(dwp[i],p[i])
                  + CrossProduct(dw[i],CrossProduct(w[i],p[i]))
                  + CrossProduct(w[i],CrossProduct(dw[i],p[i]))
                  + (CrossProduct(wp[i],dp[i])
                     + CrossProduct(w[i],CrossProduct(w[i],dp[i])))
                  *dq(i);
      }
      a[i] = CrossProduct(wp[i],links[i].r)
             + CrossProduct(w[i],CrossProduct(w[i],links[i].r))
             + vp[i];
      da[i] = CrossProduct(dwp[i],links[i].r)
              + CrossProduct(dw[i],CrossProduct(w[i],links[i].r))
              + CrossProduct(w[i],CrossProduct(dw[i],links[i].r))
              + dvp[i];
   }

   for(i = dof; i >= 1; i--)
   {
      F[i] = a[i] * links[i].m;
      dF[i] = da[i] * links[i].m;
      N[i] = links[i].I*wp[i] + CrossProduct(w[i],links[i].I*w[i]);
      dN[i] = links[i].I*dwp[i] + CrossProduct(dw[i],links[i].I*w[i])
              + CrossProduct(w[i],links[i].I*dw[i]);
      if(i == dof)
      {
         f[i] = F[i];
         df[i] = dF[i];
         n[i] = CrossProduct(p[i],f[i])
                + CrossProduct(links[i].r,F[i]) + N[i];
         dn[i] = CrossProduct(p[i],df[i])
                 + CrossProduct(links[i].r,dF[i]) + dN[i];
         if(links[i].get_joint_type() != 0)
            dn[i] += CrossProduct(dp[i],f[i])*dq(i);
      }
      else
      {
         f[i] = links[i+1].R*f[i+1] + F[i];
         df[i] = links[i+1].R*df[i+1] + dF[i];
         if(links[i].get_joint_type() == 0)
            df[i] += Q*links[i+1].R*f[i+1]*dq(i+1);
         n[i] = links[i+1].R*n[i+1] + CrossProduct(p[i],f[i])
                + CrossProduct(links[i].r,F[i]) + N[i];
         dn[i] = links[i+1].R*dn[i+1] + CrossProduct(p[i],df[i])
                 + CrossProduct(links[i].r,dF[i]) + dN[i];
         if(links[i].get_joint_type() == 0)
            dn[i] += Q*links[i+1].R*n[i+1]*dq(i+1);
         else
            dn[i] += CrossProduct(dp[i],f[i])*dq(i);
      }

      if(links[i].get_joint_type() == 0)
      {
         temp = ((z0.t()*links[i].R)*n[i]);
         ltorque(i) = temp(1,1);
         temp = ((z0.t()*links[i].R)*dn[i]);
         dtorque(i) = temp(1,1);
      }
      else
      {
         temp = ((z0.t()*links[i].R)*f[i]);
         ltorque(i) = temp(1,1);
         temp = ((z0.t()*links[i].R)*df[i]);
         dtorque(i) = temp(1,1);
      }
   }
}


void mRobot::delta_torque(const ColumnVector & q, const ColumnVector & qp,
                          const ColumnVector & qpp, const ColumnVector & dq,
                          const ColumnVector & dqp, const ColumnVector & dqpp,
                          ColumnVector & ltorque, ColumnVector & dtorque)
/*!
  @brief Delta torque dynamics.

  This function computes
  \f[
    \delta \tau = D(q) \delta \ddot{q}
    + S_1(q,\dot{q}) \delta \dot{q} + S_2(q,\dot{q},\ddot{q}) \delta q    
  \f]

  Murray and Neuman Cite_: Murray86 have developed an efficient recursive 
  linearized Newton-Euler formulation.   In order to apply the RNE as presented in 
  let us define the following variables 

  \f[
    p_{di} = \frac{\partial p_i}{\partial d_i} = 
    \left [
      \begin{array}{ccc}
        0 & \sin \alpha_i & \cos \alpha_i 
      \end{array}
    \right ]^T
  \f]
  \f[
    Q = 
    \left [
      \begin{array}{ccc}
        0 & -1 & 0  \\
	1 & 0 & 0  \\
	0 & 0 & 0 
      \end{array}
  \right ]
  \f]

  Forward Iterations for \f$i=1, 2, \ldots, n\f$. 
  Initialize: \f$\delta \omega_0 = \delta \dot{\omega}_0 = \delta \dot{v}_0 = 0\f$.

  \f[
  \delta \omega_i = R_i^T \delta \omega_{i-1} + \sigma_i
  (z_0 \delta \dot{\theta}_i - QR_i^T \omega_i \delta \theta_i) 
  \f]
  \f[
  \delta \dot{\omega}_i = R_i^T \delta \dot{w}_{i-1} +
  \sigma_i [R_i^T \delta \omega_{i-1} \times z_0 \dot{\theta}_i
  + R_i^T \omega_{i-1} \times z_0 \delta \dot{\theta}_i  
  + z_0 \ddot{\theta}_i - (QR_i^T \dot{\omega}_{i-1} + QR_i^T \omega_{i-1}
  \times \omega{z}_0 \dot{\theta}_i) \delta \theta_i ] 
  \f]
  \f[
  \delta \dot{v}_i = R_i^T\Big(\delta \dot{\omega}_{i-1} \times p_i + \delta \omega_{i-1}
  \times(\omega_{i-1} \times p_i) + \omega_{i-1} \times( \delta \omega_{i-1} \times p_i) +
  \delta \dot{v}_i\Big) + (1-\sigma_i)\Big(2\delta \omega_i \times z_0\dot{d}_i
  + 2\omega_i \times z_0 \delta \dot{d}_i + z_0 \delta \ddot{d}_i\Big) 
  - \sigma_i QR_i^T \Big(\dot{\omega}_{i-1} \times p_i + \omega_{i-1} \times(w_{i-1}\times p_i)
  + \dot{v}_i\Big) \delta \theta_i + (1-\sigma_i) R_i^T \Big(\dot{\omega}_{i-1} \times
  p_{di} + \omega_{i-1} \times(\omega_{i-1}\times p_{di})\Big) \delta d_i
  \f]
  
  Backward Iterations for \f$i=n, n-1, \ldots, 1\f$. 
  Initialize: \f$\delta f_{n+1} = \delta n_{n+1} = 0\f$.

  \f[
  \delta \dot{v}_{ci} = \delta \dot{v}_i + \delta \dot{\omega}_i\times
  r_i + \delta \omega_i \times(\omega_i \times r_i)
  + \omega_i \times ( \delta \omega_i \times r_i) 
  \f]
  \f[
  \delta F_i = m_i \delta \dot{v}_{ci}
  \f]
  \f[
  \delta N_i = I_{ci} \delta \dot{\omega}_i + \delta
  \omega_i \times (I_{ci}\omega_i) + \omega_i \times
  (I_{ci} \delta \omega_i)
  \f]
  \f[
  \delta f_i = R_{i+1} \delta f_{i+1} +
  \delta F_i + \sigma_{i+1} R_{i+1} Qf_{i+1} \delta \theta_{i+1}
  \f]
  \f[
  \delta n_i = \delta N_i + R_{i+1} \delta n_{i+1} + r_i\times \delta F_i
  + p_{i+1} \times R_{i+1} \delta f_{i+1} 
  + \sigma_{i+1} \Big( R_{i+1} Qn_{i+1}
  + p_{i+1} \times R_{i+1} Qf_{i+1} \Big) \delta
  \theta_{i+1} + (1-\sigma_{i+1} ) p_{di+1} p_{di+1} \times R_{i+1}
  f_{i+1} \delta d_{i+1}
  \f]
  \f[
  \delta \tau_i = \sigma \delta n_i^T z_0 +
  (1 - \sigma_i) \delta f_i^T z_0
  \f]
*/
{
   int i;
   Matrix Rt, temp;
   if(q.Ncols() != 1 || q.Nrows() != dof) error("q has wrong dimension");
   if(qp.Ncols() != 1 || qp.Nrows() != dof) error("qp has wrong dimension");
   if(qpp.Ncols() != 1 || qpp.Nrows() != dof) error("qpp has wrong dimension");
   if(dq.Ncols() != 1 || dq.Nrows() != dof) error("dq has wrong dimension");
   if(dqp.Ncols() != 1 || dqp.Nrows() != dof) error("dqp has wrong dimension");
   if(dqpp.Ncols() != 1 || dqpp.Nrows() != dof) error("dqpp has wrong dimension");
   ltorque = ColumnVector(dof);
   dtorque = ColumnVector(dof);
   set_q(q);

   vp[0] = gravity;
   ColumnVector z0(3);
   z0(1) = 0.0; z0(2) = 0.0; z0(3) = 1.0;
   Matrix Q(3,3);
   Q = 0.0;
   Q(1,2) = -1.0;
   Q(2,1) = 1.0;
   for(i = 1; i <= dof; i++)
   {
      Rt = links[i].R.t();
      p[i] = links[i].p;
      if(links[i].get_joint_type() != 0)
      {
         dp[i] = ColumnVector(3);
         dp[i](1) = 0.0;
         dp[i](2) = Rt(2,3);
         dp[i](3) = Rt(3,3);
      }
      if(links[i].get_joint_type() == 0)
      {
         w[i] = Rt*w[i-1] + z0*qp(i);
         dw[i] = Rt*dw[i-1] + z0*dqp(i)
                 - Q*Rt*w[i-1]*dq(i);
         wp[i] = Rt*wp[i-1] + CrossProduct(Rt*w[i-1],z0*qp(i))
                 + z0*qpp(i);
         dwp[i] = Rt*dwp[i-1] + CrossProduct(Rt*dw[i-1],z0*qp(i))
                  + CrossProduct(Rt*w[i-1],z0*dqp(i))
                  - (Q*Rt*wp[i-1] + CrossProduct(Q*Rt*w[i-1],z0*qp(i)))*dq(i)
                  + z0*dqpp(i);
         vp[i] = Rt*(CrossProduct(wp[i-1],p[i])
                     + CrossProduct(w[i-1],CrossProduct(w[i-1],p[i]))
                     + vp[i-1]);
         dvp[i] = Rt*(CrossProduct(dwp[i-1],p[i])
                      + CrossProduct(dw[i-1],CrossProduct(w[i-1],p[i]))
                      + CrossProduct(w[i-1],CrossProduct(dw[i-1],p[i])) + dvp[i-1])
                  - Q*Rt*(CrossProduct(wp[i-1],p[i])
                          + CrossProduct(w[i-1],CrossProduct(w[i-1],p[i])) + vp[i-1])*dq(i);
      }
      else
      {
         w[i] = Rt*w[i-1];
         dw[i] = Rt*dw[i-1];
         wp[i] = Rt*wp[i-1];
         dwp[i] = Rt*dwp[i-1];
         vp[i] = Rt*(vp[i-1] + CrossProduct(wp[i-1],p[i])
                     + CrossProduct(w[i-1],CrossProduct(w[i-1],p[i])))
                 + z0*qpp(i) + 2.0*CrossProduct(w[i],z0*qp(i));

         dvp[i] = Rt*(CrossProduct(dwp[i-1],p[i])
                      + CrossProduct(dw[i-1],CrossProduct(w[i-1],p[i]))
                      + CrossProduct(w[i-1],CrossProduct(dw[i-1],p[i])) + dvp[i-1]
                      + (CrossProduct(wp[i-1],dp[i])
                         + CrossProduct(w[i-1],CrossProduct(w[i-1],dp[i])))*dq(i))
                  + 2*(CrossProduct(dw[i],z0*qp(i)) + CrossProduct(w[i],z0*dqp(i)))
                  + z0*dqpp(i);
      }
      a[i] = CrossProduct(wp[i],links[i].r)
             + CrossProduct(w[i],CrossProduct(w[i],links[i].r))
             + vp[i];
      da[i] = CrossProduct(dwp[i],links[i].r)
              + CrossProduct(dw[i],CrossProduct(w[i],links[i].r))
              + CrossProduct(w[i],CrossProduct(dw[i],links[i].r))
              + dvp[i];
   }

   for(i = dof; i >= 1; i--) {
      F[i] = a[i] * links[i].m;
      N[i] = links[i].I*wp[i] + CrossProduct(w[i],links[i].I*w[i]);
      dF[i] = da[i] * links[i].m;
      dN[i] = links[i].I*dwp[i] + CrossProduct(dw[i],links[i].I*w[i])
              + CrossProduct(w[i],links[i].I*dw[i]);

      if(i == dof)
      {
         f[i] = F[i];
         df[i] = dF[i];
         n[i] = CrossProduct(links[i].r,F[i]) + N[i];
         dn[i] = dN[i] + CrossProduct(links[i].r,dF[i]);
      }
      else
      {
         f[i] = links[i+1].R*f[i+1] + F[i];
         df[i] = links[i+1].R*df[i+1] + dF[i];
         if(links[i+1].get_joint_type() == 0)
            df[i] += links[i+1].R*Q*f[i+1]*dq(i+1);

         n[i] = links[i+1].R*n[i+1] + CrossProduct(p[i+1],links[i+1].R*f[i+1])
                + CrossProduct(links[i].r,F[i]) + N[i];
         dn[i] = links[i+1].R*dn[i+1] + CrossProduct(p[i+1],links[i+1].R*df[i+1])
                 + CrossProduct(links[i].r,dF[i]) + dN[i];
         if(links[i+1].get_joint_type() == 0)
            dn[i] += (links[i+1].R*Q*n[i+1] +
                      CrossProduct(p[i+1],links[i+1].R*Q*f[i+1]))*dq(i+1);
         else
            dn[i] += CrossProduct(dp[i+1],links[i+1].R*f[i+1])*dq(i+1);
      }

      if(links[i].get_joint_type() == 0)
      {
         temp = z0.t()*n[i];
         ltorque(i) = temp(1,1);
         temp = z0.t()*dn[i];
         dtorque(i) = temp(1,1);
      }
      else
      {
         temp = z0.t()*f[i];
         ltorque(i) = temp(1,1);
         temp = z0.t()*df[i];
         dtorque(i) = temp(1,1);
      }
   }
}

void mRobot_min_para::delta_torque(const ColumnVector & q, const ColumnVector & qp,
                                   const ColumnVector & qpp, const ColumnVector & dq,
                                   const ColumnVector & dqp, const ColumnVector & dqpp,
                                   ColumnVector & ltorque, ColumnVector & dtorque)
/*!
  @brief Delta torque dynamics.
  
  See mRobot::delta_torque for equations.
*/
{
   int i;
   Matrix Rt, temp;
   if(q.Ncols() != 1 || q.Nrows() != dof) error("q has wrong dimension");
   if(qp.Ncols() != 1 || qp.Nrows() != dof) error("qp has wrong dimension");
   if(qpp.Ncols() != 1 || qpp.Nrows() != dof) error("qpp has wrong dimension");
   if(dq.Ncols() != 1 || dq.Nrows() != dof) error("dq has wrong dimension");
   if(dqp.Ncols() != 1 || dqp.Nrows() != dof) error("dqp has wrong dimension");
   if(dqpp.Ncols() != 1 || dqpp.Nrows() != dof) error("dqpp has wrong dimension");
   ltorque = ColumnVector(dof);
   dtorque = ColumnVector(dof);
   set_q(q);

   vp[0] = gravity;
   ColumnVector z0(3);
   z0(1) = 0.0; z0(2) = 0.0; z0(3) = 1.0;
   Matrix Q(3,3);
   Q = 0.0;
   Q(1,2) = -1.0;
   Q(2,1) = 1.0;
   for(i = 1; i <= dof; i++)
   {
      Rt = links[i].R.t();
      p[i] = links[i].p;
      if(links[i].get_joint_type() != 0)
      {
         dp[i] = ColumnVector(3);
         dp[i](1) = 0.0;
         dp[i](2) = Rt(2,3);
         dp[i](3) = Rt(3,3);
      }
      if(links[i].get_joint_type() == 0)
      {
         w[i] = Rt*w[i-1] + z0*qp(i);
         dw[i] = Rt*dw[i-1] + z0*dqp(i)
                 - Q*Rt*w[i-1]*dq(i);
         wp[i] = Rt*wp[i-1] + CrossProduct(Rt*w[i-1],z0*qp(i))
                 + z0*qpp(i);
         dwp[i] = Rt*dwp[i-1] + CrossProduct(Rt*dw[i-1],z0*qp(i))
                  + CrossProduct(Rt*w[i-1],z0*dqp(i))
                  - (Q*Rt*wp[i-1] + CrossProduct(Q*Rt*w[i-1],z0*qp(i)))*dq(i)
                  + z0*dqpp(i);
         vp[i] = Rt*(CrossProduct(wp[i-1],p[i])
                     + CrossProduct(w[i-1],CrossProduct(w[i-1],p[i]))
                     + vp[i-1]);
         dvp[i] = Rt*(CrossProduct(dwp[i-1],p[i])
                      + CrossProduct(dw[i-1],CrossProduct(w[i-1],p[i]))
                      + CrossProduct(w[i-1],CrossProduct(dw[i-1],p[i])) + dvp[i-1])
                  - Q*Rt*(CrossProduct(wp[i-1],p[i])
                          + CrossProduct(w[i-1],CrossProduct(w[i-1],p[i])) + vp[i-1])*dq(i);
      }
      else
      {
         w[i] = Rt*w[i-1];
         dw[i] = Rt*dw[i-1];
         wp[i] = Rt*wp[i-1];
         dwp[i] = Rt*dwp[i-1];
         vp[i] = Rt*(vp[i-1] + CrossProduct(wp[i-1],p[i])
                     + CrossProduct(w[i-1],CrossProduct(w[i-1],p[i])))
                 + z0*qpp(i)+ 2.0*CrossProduct(w[i],z0*qp(i));
         dvp[i] = Rt*(CrossProduct(dwp[i-1],p[i])
                      + CrossProduct(dw[i-1],CrossProduct(w[i-1],p[i]))
                      + CrossProduct(w[i-1],CrossProduct(dw[i-1],p[i])) + dvp[i-1]
                      + (CrossProduct(wp[i-1],dp[i])
                         + CrossProduct(w[i-1],CrossProduct(w[i-1],dp[i])))*dq(i))
                  + 2*(CrossProduct(dw[i],z0*qp(i)) + CrossProduct(w[i],z0*dqp(i)))
                  + z0*dqpp(i);
      }
   }

   for(i = dof; i >= 1; i--) {
      F[i] = vp[i]*links[i].m + CrossProduct(wp[i], links[i].mc) +
             CrossProduct(w[i], CrossProduct(w[i], links[i].mc));
      dF[i] = dvp[i]*links[i].m + CrossProduct(dwp[i],links[i].mc)
              + CrossProduct(dw[i],CrossProduct(w[i],links[i].mc))
              + CrossProduct(w[i],CrossProduct(dw[i],links[i].mc));
      N[i] = links[i].I*wp[i] + CrossProduct(w[i],links[i].I*w[i])
             - CrossProduct(vp[i], links[i].mc);
      dN[i] = links[i].I*dwp[i] + CrossProduct(dw[i],links[i].I*w[i])
              + CrossProduct(w[i],links[i].I*dw[i])
              - CrossProduct(dvp[i],links[i].mc);

      if(i == dof)
      {
         f[i] = F[i];
         df[i] = dF[i];
         n[i] = N[i];
         dn[i] = dN[i];
      }
      else
      {
         f[i] = links[i+1].R*f[i+1] + F[i];
         df[i] = links[i+1].R*df[i+1] + dF[i];
         if(links[i+1].get_joint_type() == 0)
            df[i] += links[i+1].R*Q*f[i+1]*dq(i+1);

         n[i] = links[i+1].R*n[i+1] + CrossProduct(p[i+1],links[i+1].R*f[i+1])
                + N[i];
         dn[i] = links[i+1].R*dn[i+1] + CrossProduct(p[i+1],links[i+1].R*df[i+1])
                 + dN[i];
         if(links[i+1].get_joint_type() == 0)
            dn[i] += (links[i+1].R*Q*n[i+1] +
                      CrossProduct(p[i+1],links[i+1].R*Q*f[i+1]))*dq(i+1);
         else
            dn[i] += CrossProduct(dp[i+1],links[i+1].R*f[i+1])*dq(i+1);
      }

      if(links[i].get_joint_type() == 0)
      {
         temp = z0.t()*n[i];
         ltorque(i) = temp(1,1);
         temp = z0.t()*dn[i];
         dtorque(i) = temp(1,1);
      }
      else
      {
         temp = z0.t()*f[i];
         ltorque(i) = temp(1,1);
         temp = z0.t()*df[i];
         dtorque(i) = temp(1,1);
      }
   }
}

#ifdef use_namespace
}
#endif
