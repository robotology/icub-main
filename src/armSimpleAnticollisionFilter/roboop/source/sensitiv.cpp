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

2004/07/01: Ethan Tira-Thompson
    -Added support for newmat's use_namespace #define, using ROBOOP namespace

2004/07/02: Etienne Lachance
    -Added Doxygen comments.
-------------------------------------------------------------------------------
*/

/*!
  @file sensitiv.cpp
  @brief Delta torque (linearized dynamics).
*/

//! @brief RCS/CVS version.
static const char rcsid[] = "$Id: sensitiv.cpp,v 1.1 2007/07/24 16:03:10 amaldo Exp $";

#include "robot.h"

#ifdef use_namespace
namespace ROBOOP {
  using namespace NEWMAT;
#endif


ReturnMatrix Robot_basic::dtau_dq(const ColumnVector & q,
                                  const ColumnVector & qp,
                                  const ColumnVector & qpp)
/*!
  @brief Sensitivity of the dynamics with respect to \f$ q \f$

  This function computes \f$S_2(q, \dot{q}, \ddot{q})\f$.
*/
{
   int i, j;
   Matrix ldtau_dq(dof,dof);
   ColumnVector ltorque(dof);
   ColumnVector dtorque(dof);
   ColumnVector dq(dof);
   for(i = 1; i <= dof; i++) {
      for(j = 1; j <= dof; j++) {
         dq(j) = (i == j ? 1.0 : 0.0);
      }
      dq_torque(q,qp,qpp,dq,ltorque,dtorque);
      for(j = 1; j <= dof; j++) {
         ldtau_dq(j,i) = dtorque(j);
      }
   }
   ldtau_dq.Release(); return ldtau_dq;
}

ReturnMatrix Robot_basic::dtau_dqp(const ColumnVector & q,
                                   const ColumnVector & qp)
/*!
  @brief Sensitivity of the dynamics with respect to \f$\dot{q} \f$

  This function computes \f$S_1(q, \dot{q})\f$.
*/
{
   int i, j;
   Matrix ldtau_dqp(dof,dof);
   ColumnVector ltorque(dof);
   ColumnVector dtorque(dof);
   ColumnVector dqp(dof);
   for(i = 1; i <= dof; i++) {
      for(j = 1; j <= dof; j++) {
         dqp(j) = (i == j ? 1.0 : 0.0);
      }
      dqp_torque(q,qp,dqp,ltorque,dtorque);
      for(j = 1; j <= dof; j++) {
         ldtau_dqp(j,i) = dtorque(j);
      }
   }
   ldtau_dqp.Release(); return ldtau_dqp;
}

#ifdef use_namespace
}
#endif



