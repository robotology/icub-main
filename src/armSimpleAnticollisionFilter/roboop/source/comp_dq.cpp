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

2003/14/05: Etienne Lachance
   -Fix Robot::dq_torque.
   -Added mRobot/mRobot_min_para::dq_torque.

2004/07/01: Etienne Lachance
   -Replace vec_x_prod by CrossProduct.

2004/07/01: Ethan Tira-Thompson
    -Added support for newmat's use_namespace #define, using ROBOOP namespace

2004/07/02: Etienne Lachance
   -Added Doxygen comments.
-------------------------------------------------------------------------------
*/

/*!
  @file comp_dq.cpp
  @brief Delta torque (linearized dynamics).
*/

//! @brief RCS/CVS version.
static const char rcsid[] = "$Id: comp_dq.cpp,v 1.1 2007/07/24 16:03:09 amaldo Exp $";

#include "robot.h"

#ifdef use_namespace
namespace ROBOOP {
  using namespace NEWMAT;
#endif


void Robot::dq_torque(const ColumnVector & q, const ColumnVector & qp,
                      const ColumnVector & qpp, const ColumnVector & dq,
                      ColumnVector & ltorque, ColumnVector & dtorque)
/*!
  @brief Delta torque due to delta joint position.

  This function computes \f$S_2(q, \dot{q}, \ddot{q})\delta q\f$.
  See Robot::delta_torque for equations.
*/
{
   int i;
   Matrix Rt, temp;
   if(q.Ncols() != 1 || q.Nrows() != dof) error("q has wrong dimension");
   if(qp.Ncols() != 1 || qp.Nrows() != dof) error("qp has wrong dimension");
   if(qpp.Ncols() != 1 || qpp.Nrows() != dof) error("qpp has wrong dimension");
   if(dq.Ncols() != 1 || qpp.Nrows() != dof) error("dq has wrong dimension");
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
         dw[i] = Rt*(dw[i-1] - Q*w[i-1]*dq(i));
         wp[i] = Rt*(wp[i-1] + z0*qpp(i)
                     + CrossProduct(w[i-1],z0*qp(i)));
         dwp[i] = Rt*(dwp[i-1]
                      + CrossProduct(dw[i-1],z0*qp(i))
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
                     + 2.0*CrossProduct(w[i],Rt*z0*qp(i)))
                 + CrossProduct(wp[i],p[i])
                 + CrossProduct(w[i],CrossProduct(w[i],p[i]));
         dvp[i] = Rt*(dvp[i-1]
                      + 2.0*CrossProduct(dw[i-1],z0*qp(i)))
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
      N[i] = links[i].I*wp[i] + CrossProduct(w[i],links[i].I*w[i]);
      dF[i] = da[i] * links[i].m;
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
         if(links[i+1].get_joint_type() == 0)
            df[i] += Q*links[i+1].R*f[i+1]*dq(i+1);
         n[i] = links[i+1].R*n[i+1] + CrossProduct(p[i],f[i])
                + CrossProduct(links[i].r,F[i]) + N[i];

         dn[i] = links[i+1].R*dn[i+1] + CrossProduct(p[i],df[i])
                 + CrossProduct(links[i].r,dF[i]) + dN[i];
         if(links[i+1].get_joint_type() == 0)
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


void mRobot::dq_torque(const ColumnVector & q, const ColumnVector & qp,
                       const ColumnVector & qpp, const ColumnVector & dq,
                       ColumnVector & ltorque, ColumnVector & dtorque)
/*!
  @brief Delta torque due to delta joint position.

  This function computes \f$S_2(q, \dot{q}, \ddot{q})\delta q\f$.
  See mRobot::delta_torque for equations.
*/
{
   int i;
   Matrix Rt, temp;
   if(q.Ncols() != 1 || q.Nrows() != dof) error("q has wrong dimension");
   if(qp.Ncols() != 1 || qp.Nrows() != dof) error("qp has wrong dimension");
   if(qpp.Ncols() != 1 || qpp.Nrows() != dof) error("qpp has wrong dimension");
   if(dq.Ncols() != 1 || dq.Nrows() != dof) error("dq has wrong dimension");
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
         dw[i] = Rt*dw[i-1] - Q*Rt*w[i-1]*dq(i);
         wp[i] = Rt*wp[i-1] + CrossProduct(Rt*w[i-1],z0*qp(i))
                 + z0*qpp(i);
         dwp[i] = Rt*dwp[i-1] + CrossProduct(Rt*dw[i-1],z0*qp(i))
                  - (Q*Rt*wp[i-1] + CrossProduct(Q*Rt*w[i-1],z0*qp(i)))*dq(i);
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
                  + 2*CrossProduct(dw[i],z0*qp(i));
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


void mRobot_min_para::dq_torque(const ColumnVector & q, const ColumnVector & qp,
                                const ColumnVector & qpp, const ColumnVector & dq,
                                ColumnVector & ltorque, ColumnVector & dtorque)
/*!
  @brief Delta torque due to delta joint position.

  This function computes \f$S_2(q, \dot{q}, \ddot{q})\delta q\f$.
  See mRobot::delta_torque for equations.
*/
{
   int i;
   Matrix Rt, temp;
   if(q.Ncols() != 1 || q.Nrows() != dof) error("q has wrong dimension");
   if(qp.Ncols() != 1 || qp.Nrows() != dof) error("qp has wrong dimension");
   if(qpp.Ncols() != 1 || qpp.Nrows() != dof) error("qpp has wrong dimension");
   if(dq.Ncols() != 1 || dq.Nrows() != dof) error("dq has wrong dimension");
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
         dw[i] = Rt*dw[i-1] - Q*Rt*w[i-1]*dq(i);
         wp[i] = Rt*wp[i-1] + CrossProduct(Rt*w[i-1],z0*qp(i))
                 + z0*qpp(i);
         dwp[i] = Rt*dwp[i-1] + CrossProduct(Rt*dw[i-1],z0*qp(i))
                  - (Q*Rt*wp[i-1] + CrossProduct(Q*Rt*w[i-1],z0*qp(i)))*dq(i);
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
                         + CrossProduct(w[i-1],CrossProduct(w[i-1],dp[i])))*dq(i));
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
