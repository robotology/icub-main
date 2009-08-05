/*
Copyright (C) 2004  Samuel Bélanger

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

email: samuel.belanger@polymtl.ca or richard.gourdeau@polymtl.ca
*/

#ifndef __cplusplus
#error Must use C++
#endif
#ifndef STEWART_H
#define STEWART_H

/*!
  @file stewart.h
  @brief Stewart class definitions.
*/

//! @brief RCS/CVS version.
static const char header_stewart_rcsid[] = "$Id: stewart.h,v 1.1 2007/07/24 16:03:10 amaldo Exp $";

#include "utils.h"

#ifdef use_namespace
namespace ROBOOP {
  using namespace NEWMAT;
#endif


/*!
  @class LinkStewart
  @brief LinkStewart definitions.

  A Stewart platform is composed 6 links.  This class describe the proprities
  of each of the platform's link.
*/
class LinkStewart {
    friend class Stewart;	
private:
    ColumnVector ap,	//!< Platform coordinates of the link in the local frame.
	         b;     //!< Base coordinates of the link int the global frame.
    Real I1aa,          //!< Inertia along the coaxial axis for part 1.
	 I1nn,		//!< Inertia along the tangent axis for part 1.
	 I2aa,		//!< Inertia along the coaxial axis for part 2.
	 I2nn,		//!< Inertia along the tangent axis for part 2.
	 m1,		//!< Mass of part 1.
	 m2,		//!< Mass of part 2.
	 Lenght1,	//!< Lenght between the mass center (part 1) and the platform attachment.
	 Lenght2;	//!< Lenght between the mass center (part 2) and the base attachment.
public:

    ColumnVector UnitV,	 //!< Unit Vector of the link.
                 aPos,   //!< Position of the platform attachment point.
		 Vu,	 //!< Unit Vector of the universal joint (Rotational).
                 Vc,	 //!< Unit Vector of the universal joint (Rotational).
                 Vv,	 //!< Unit Vector of the universal joint (Rotational).
                 da,	 //!< Speed of the platform attachment point .
                 dda,	 //!< Acceleration of the platform attachment point.
                 LOmega, //!< Angular speed of the link.
                 LAlpha, //!< Angular acceleration of the link.
                 ACM1,	 //!< Acceleration of the first center of mass.
                 M,	 //!< Moment vector of the link.
                 N,      //!< Intermediate vector for dynamics calculations .
                 gravity;//!< Gravity vector.
    Real L;	         //!< Lenght of the link.
	
    LinkStewart (const ColumnVector & InitLink, const Matrix wRp, const ColumnVector q);
    LinkStewart (const LinkStewart & x);
    LinkStewart ();

    ~LinkStewart ();
    const LinkStewart & operator = (const LinkStewart & x);

    void set_ap (const ColumnVector NewAp); 
    void set_b (const ColumnVector Newb); 
    void set_I1aa(const Real NewI1aa);
    void set_I1nn (const Real NewI1nn);
    void set_I2aa (const Real NewI2aa);
    void set_I2nn (const Real NewI2nn);
    void set_m1 (const Real Newm1);
    void set_m2 (const Real Newm2);
    void set_Lenght1 (const Real NewLenght1);
    void set_Lenght2 (const Real NewLenght2);

    ReturnMatrix get_ap() const;
    ReturnMatrix get_b() const;
    Real get_I1aa () const;
    Real get_I1nn () const;
    Real get_I2aa () const;
    Real get_I2nn () const;
    Real get_m1 () const;
    Real get_m2 () const;
    Real get_Lenght1() const;
    Real get_Lenght2 () const;

    void LTransform(const Matrix wRp, const ColumnVector q);
    void d_LTransform(const ColumnVector dq, const ColumnVector Omega, const Real dl, 
		      const Real ddl);
    void dd_LTransform(const ColumnVector ddq, const ColumnVector Omega, 
		       const ColumnVector Alpha, const Real dl, const Real ddl);
    void tau_LTransform(const Real dl, const Real ddl, const Real Gravity);
    ReturnMatrix Find_UnitV ();
    ReturnMatrix Find_a (const Matrix _wRp, const ColumnVector _q);
    ReturnMatrix Find_da (const ColumnVector dq,const ColumnVector Omega);	
    ReturnMatrix Find_dda (const ColumnVector ddq, const ColumnVector Omega, 
			   const ColumnVector Alpha);
    Real Find_Lenght ();

    ReturnMatrix Find_VctU ();
    ReturnMatrix Find_VctV ();
    ReturnMatrix Find_VctC ();
    ReturnMatrix Find_AngularKin (const Real dl, const Real ddl);
    ReturnMatrix NormalForce();
    ReturnMatrix AxialForce (const Matrix J1, const ColumnVector C, const int Index);    
    ReturnMatrix Find_N(const Real Gravity = GRAVITY);
    ReturnMatrix Moment();
    Real ActuationForce (const Matrix J1, const ColumnVector C, const int Index,
			 const Real Gravity = GRAVITY);
    ReturnMatrix Find_ACM1 (const Real dl, const Real ddl);
};

/*!
  @class Stewart
  @brief Stewart definitions.

*/
class Stewart {

private:
    bool UJointAtBase;	//!< Gives the position of the universal joint (true if at base, false if at platform)
    ColumnVector q,      //!< Platform position (xyz + euler angles)
                 dq,     //!< Platform speed
                 ddq,    //!< Platform acceleration
                 pR,     //!< Platform center of mass (in its own referential)
                 gravity;//!< Gravity vector.
    Matrix       pIp;   //!< Platform Inertia (local ref.)
    Real mp,            //!< Platform mass
         p,             //!< Pitch of the ballscrew (links)
         n,             //!< Gear ratio	(links motor)
         Js,            //!< Moment of inertia (ballscrew)
         Jm,            //!< Moment of inertia (motor)
         bs,            //!< Viscous damping coefficient of the ballscrew
         bm,            //!< Viscous damping coefficient of the motor
         Kb,            //!< Motor back EMF
         L,             //!< Motor Inductance
         R,             //!< Motor armature resistance
         Kt;            //!< Motor torque 
    LinkStewart Links[6]; //!< Platform links	

 public:
    Matrix wRp,         //!< Rotation matrix describing the orientation of the platform 
           Jacobian,    //!< Jacobian matrix
           IJ1,         //!< Inverse of the first intermediate Jacobian matrix
           IJ2;         //!< Inverse of the second intermediate Jacobian matrix
    ColumnVector dl,    //!< Rate of expension vector
                 ddl,   //!< Acceleration of expension vector
	         Alpha, //!< Angular speed of the platform
	         Omega; //!< Angular acceleration of the platform

    Stewart ();
    Stewart (const Matrix InitPlat, bool Joint = true);
    Stewart (const Stewart & x);
    Stewart (const std::string & filename, const std::string & PlatformName);
    ~Stewart ();
    const Stewart & operator = (const Stewart& x);				

    void set_Joint(const bool _Joint);
    void set_q (const ColumnVector _q);
    void set_dq (const ColumnVector _dq);
    void set_ddq (const ColumnVector _ddq);
    void set_pR (const ColumnVector _pR);
    void set_pIp (const Matrix _pIp);
    void set_mp (const Real _mp);
    bool get_Joint () const;
    ReturnMatrix get_q() const;
    ReturnMatrix get_dq() const;
    ReturnMatrix get_ddq() const;
    ReturnMatrix get_pR() const;
    ReturnMatrix get_pIp() const;
    Real get_mp () const;
    
    void Transform();
    ReturnMatrix Find_wRp ();
    ReturnMatrix Find_Omega ();
    ReturnMatrix Find_Alpha ();
    ReturnMatrix jacobian ();
    ReturnMatrix Find_InvJacob1 ();
    ReturnMatrix Find_InvJacob2 ();
    ReturnMatrix jacobian_dot();
    ReturnMatrix Find_dl ();
    ReturnMatrix Find_ddl ();
    ReturnMatrix Find_C(const Real Gravity = GRAVITY);
    ReturnMatrix Torque(const Real Gravity = GRAVITY);
    ReturnMatrix JointSpaceForceVct(const Real Gravity = GRAVITY);
    ReturnMatrix InvPosKine();
    ReturnMatrix ForwardKine(const ColumnVector guess_q, const ColumnVector l_given,
			     const Real tolerance = 0.001);
    ReturnMatrix Find_h(const Real Gravity = GRAVITY);
    ReturnMatrix Find_M();
    ReturnMatrix ForwardDyn(const ColumnVector Torque, const Real Gravity=GRAVITY);
    void Find_Mc_Nc_Gc(Matrix & Mc, Matrix & Nc, Matrix & Gc);
    ReturnMatrix ForwardDyn_AD(const ColumnVector Command, const Real t);
};

#ifdef use_namespace
}
#endif

#endif //Class Stewart
