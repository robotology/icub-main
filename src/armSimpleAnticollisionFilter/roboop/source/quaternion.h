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

[1] J.C.K. Chou, "Quaternion Kinematic and Dynamic Differential Equations", 
    IEEE Transaction on Robotics and Automation, vol 8, p53-64.

[2] S. Chiaverini, B. Siciliano, "The Unit Quaternion: A Useful Tool for
    Inverse Kinematics of Robot Manipulators", Systems Analysis, Modelling
    and Simulation, vol. 35, pp.45-60, 1999.

[3] C. Natale, "Quaternion-Based Representation of Rigid Bodies Orientation",
    PRISMA LAB, PRISMA Technical Report no. 97-05, Oct 1997.

[4] M. Lillholm, E.B. Dam, M. Koch, "Quaternions, Interpolation and Animation",
    Technical Report DIKU-TR-98/5, University of Copenhagen, July 1998.

[5] D. Eberly, "Quaternion Algebra and Calculus", Magic Software Inc.,
    http://www.magic-software.com, March 1999.
-------------------------------------------------------------------------------
Revision_history:

2003/05/28: Etienne Lachance
    -Added functions Slerp, Slerp_prime, Squad, Squad_prime.
    -Added the following member functions:+=, -=, *=, /=, Exp, d_dt, Ln, Ln_4, E

2004/05/21: Etienne Lachance
   -Added Doxygen comments.

2004/07/01: Ethan Tira-Thompson
    -Added support for newmat's use_namespace #define, using ROBOOP namespace

2005/11/06: Etienne Lachance
    - No need to provide a copy constructor and the assignment operator 
      (operator=) for Quaternion class. Instead we use the one provide by the
      compiler.

2005/11/13: Etienne Lachance
    - operator* and operator/ are now non-member functions when one of the
      operand is a real. With these modifications we support q2 = c * q1 and
      q2 = q1 * c
------------------------------------------------------------------------------
*/


#ifndef QUATERNION_H
#define QUATERNION_H

/*!
  @file quaternion.h
  @brief Quaternion class.
*/

//! @brief RCS/CVS version.
static const char header_quat_rcsid[] = "$Id: quaternion.h,v 1.1 2007/07/24 16:03:10 amaldo Exp $";

#include "robot.h"

#ifdef use_namespace
namespace ROBOOP {
  using namespace NEWMAT;
#endif

#define BASE_FRAME 0
#define BODY_FRAME 1
#define EPSILON 0.0000001

/*!
  @class Quaternion
  @brief Quaternion class definition.
*/
class Quaternion
{
public:
   Quaternion();
   Quaternion(const Real angle_in_rad, const ColumnVector & axis);
   Quaternion(const Real s, const Real v1, const Real v2,
              const Real v3);
   Quaternion(const Matrix & R);

   Quaternion   operator+(const Quaternion & q)const;
   Quaternion   operator-(const Quaternion & q)const;
   Quaternion   operator*(const Quaternion & q)const;
   Quaternion   operator/(const Quaternion & q)const;
   Quaternion   conjugate()const;

//   Quaternion   i()const { return conjugate(); }
   Quaternion   i()const;
   Quaternion & unit(); 
   Quaternion   exp() const;
   Quaternion   power(const Real t) const;
   Quaternion   Log() const;

   Quaternion   dot(const ColumnVector & w, const short sign)const;
   ReturnMatrix E(const short sign)const;

   Real         norm()const;
   Real         dot_prod(const Quaternion & q)const;
   Real         s()const { return s_; }        //!< Return scalar part.
   void         set_s(const Real s){ s_ = s; } //!< Set scalar part.
   ReturnMatrix v()const { return v_; }        //!< Return vector part.
   void         set_v(const ColumnVector & v); //!< Set vector part.
   ReturnMatrix R()const;
   ReturnMatrix T()const;

private:
   Real s_;         //!< Quaternion scalar part.
   ColumnVector v_; //!< Quaternion vector part.
};

// ----------------------------------------------------------------------------

Quaternion  operator*(const Real c, const Quaternion & rhs);
Quaternion  operator*(const Quaternion & lhs, const Real c);
Quaternion  operator/(const Real c, const Quaternion & rhs);
Quaternion  operator/(const Quaternion & lhs, const Real c);

ReturnMatrix Omega(const Quaternion & q, const Quaternion & q_dot);

short Integ_quat(Quaternion & dquat_present, Quaternion & dquat_past,
                 Quaternion & quat, const Real dt);
Real Integ_Trap_quat_s(const Quaternion & present, Quaternion & past,
                       const Real dt);
ReturnMatrix Integ_Trap_quat_v(const Quaternion & present, Quaternion & past,
                               const Real dt);

Quaternion Slerp(const Quaternion & q0, const Quaternion & q1, const Real t);
Quaternion Slerp_prime(const Quaternion & q0, const Quaternion & q1, const Real t);

Quaternion Squad(const Quaternion & p, const Quaternion & a, const Quaternion & b,
                 const Quaternion & q, const Real t);
Quaternion Squad_prime(const Quaternion & p, const Quaternion & a, const Quaternion & b,
                       const Quaternion & q, const Real t);

#ifdef use_namespace
}
#endif

#endif
