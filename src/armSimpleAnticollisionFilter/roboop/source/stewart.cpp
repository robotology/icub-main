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


/*!
  @file stewart.cpp
  @brief Initialisation of Stewart platform class.
*/

//! @brief RCS/CVS version.
static const char rcsid[] = "$Id: stewart.cpp,v 1.1 2007/07/24 16:03:10 amaldo Exp $";

#include "config.h"
#include "stewart.h"

#ifdef use_namespace
namespace ROBOOP {
#endif


/*!
  @fn LinkStewart::LinkStewart()
  @brief Default Constructor.
*/
LinkStewart::LinkStewart ()
{
    b = ColumnVector(3);
    b = 0.0;
    ap = ColumnVector(3);
    ap = 0.0;
    
    I1aa = 0.0;
    I1nn = 0.0;
    I2aa = 0.0;
    I2nn = 0.0;
    m1 = 0.0;
    m2 = 0.0;
    Lenght1 = 0.0;
    Lenght2 = 0.0;
    
    ColumnVector ZeroValue(3);
    ZeroValue<<0.0<<0.0<<0.0;
    
    UnitV = ZeroValue;
    aPos = ZeroValue;
    Vu = ZeroValue;
    Vc = ZeroValue;
    Vv = ZeroValue;
    da = ZeroValue;
    dda = ZeroValue;
    LOmega = ZeroValue;
    LAlpha = ZeroValue;
    ACM1 = ZeroValue;
    N = ZeroValue;
    gravity = ZeroValue;
    L = 0.0;
}

/*!
  @fn LinkStewart::LinkStewart(const ColumnVector & InitLink, const Matrix wRp, const ColumnVector q) 
  @brief Constructor.
  @param InitLink: LinkStewart initialization matrix.
  @param wRp: Rotation matrix
  @param q: Position of the platform
*/

LinkStewart::LinkStewart (const ColumnVector & InitLink, const Matrix wRp, const ColumnVector q)
{	
    b = InitLink.Rows(1,3);
    ap = InitLink.Rows(4,6);
    I1aa = InitLink(7);
    I1nn = InitLink(8);
    I2aa = InitLink(9);
    I2nn = InitLink(10);
    m1 = InitLink(11);
    m2 = InitLink(12);
    Lenght1 = InitLink(13);
    Lenght2 = InitLink(14);

    ColumnVector ZeroValue(3);
    ZeroValue<<0.0<<0.0<<0.0;
    
    UnitV = ZeroValue;
    aPos = ZeroValue;
    Vc = ZeroValue;
    Vv = ZeroValue;
    da = ZeroValue;
    dda = ZeroValue;
    LOmega = ZeroValue;
    LAlpha = ZeroValue;
    ACM1 = ZeroValue;
    N = ZeroValue;
    gravity = ZeroValue;
    L = 0.0;
    
    aPos = Find_a(wRp,q);
    L = Find_Lenght();
    UnitV = Find_UnitV();
    Vu = Find_VctU();
}

/*!
  @fn LinkStewart::LinkStewart(const LinkStewart & x)
  @brief Copy constructor.
*/
LinkStewart::LinkStewart(const LinkStewart & x)
{
    b = x.b;
    ap = x.ap;
    I1aa = x.I1aa;
    I1nn = x.I1nn;
    I2aa = x.I2aa;
    I2nn = x.I2nn;
    m1 = x.m1;
    m2 = x.m2;
    Lenght1 = x.Lenght1;
    Lenght2 = x.Lenght2;

    UnitV = x.UnitV;
    aPos = x.aPos;
    Vu = x.Vu;
    Vc = x.Vc;
    Vv = x.Vv;
    da = x.da;
    dda = x.dda;
    LOmega = x.LOmega;
    LAlpha = x.LAlpha;
    ACM1 = x.ACM1;
    N = x.N;
    gravity = x.gravity;
    L = x.L;
}

LinkStewart::~LinkStewart()
//! @brief Destructor
{
}

const LinkStewart & LinkStewart::operator = (const LinkStewart& x)
{
    //! @brief Overload = operator.
    b = x.b;
    ap = x.ap;
    I1aa = x.I1aa;
    I1nn = x.I1nn;
    I2aa = x.I2aa;
    I2nn = x.I2nn;
    m1 = x.m1;
    m2 = x.m2;
    Lenght1 = x.Lenght1;
    Lenght2 = x.Lenght2;

    UnitV = x.UnitV;
    aPos = x.aPos;
    Vu = x.Vu;
    Vc = x.Vc;
    Vv = x.Vv;
    da = x.da;
    dda = x.dda;
    LOmega = x.LOmega;
    LAlpha = x.LAlpha;
    ACM1 = x.ACM1;
    N = x.N;
    gravity = x.gravity;
    L = x.L;
    return *this;
}

void LinkStewart::set_b(const ColumnVector  Newb)
//! @brief Set the position vector of the base attachment point.
{
    if(Newb.Nrows() == 3)
	b = Newb;
    else
	cerr<< "LinkStewart::set_b: wrong size in input vector."<< endl;
}

void LinkStewart::set_ap(const ColumnVector NewAp)
//! @brief Set the position vector of platform attachment point.
{
    if(NewAp.Nrows()== 3)
	ap = NewAp;
    else
	cerr<< "LinkStewart::set_Ap: wrong size in input vector."<< endl;
}

void LinkStewart::set_I1aa(const Real NewI1aa) 
 //! @brief Set the value of inertia along the coaxial axis of part 1.
{
    I1aa = NewI1aa;
}

void LinkStewart::set_I1nn(const Real NewI1nn) 
//! @brief Set the value of inertia along the tangent axis of part 1.
{
    I1nn = NewI1nn;
} 

void LinkStewart::set_I2aa(const Real NewI2aa) 
//! @brief Set the value of inertia along the coaxial axis of part 2.
{
    I2aa = NewI2aa;
} 

void LinkStewart::set_I2nn(const Real NewI2nn) 
//! @brief Set the value of inertia along the tangent axis of part 2.
{
    I2nn = NewI2nn;
}

void LinkStewart::set_m1(const Real Newm1) 
//! @brief  Set the mass of part 1.
{
    m1 = Newm1;
}

void LinkStewart::set_m2(const Real Newm2) 
//! @brief  Set the mass of part 2.
{
    m2 = Newm2;
}

void LinkStewart::set_Lenght1(const Real NewLenght1)
//! @brief Set the lenght between platform attachment point and center of mass of part 1.
{
    Lenght1 = NewLenght1;
}

void LinkStewart::set_Lenght2(const Real NewLenght2)
//! @brief Set the lenght between base attachment point and center of mass of part 2.
{
    Lenght2 = NewLenght2;
}

ReturnMatrix LinkStewart::get_ap() const
//! @brief Return the position vector of platform attachement point.
{
    return ap;
}

ReturnMatrix LinkStewart::get_b() const
//! @brief Return the position vector of base attachement point.
{
    return b;
}

Real LinkStewart::get_I1aa() const
//! @brief Return the value of inertia along the coaxial axis of part 1.
{
    return I1aa;
}

Real LinkStewart::get_I1nn() const
//! @brief Return the value of inertia along the tangent axis of part 1.
{
    return I1nn;
}

Real LinkStewart::get_I2aa() const
//! @brief Return the value of inertia along the coaxial axis of part 2.
{
    return I2aa;
}

Real LinkStewart::get_I2nn() const
//! @brief Return the value of inertia along the tangent axis of part 2.
{
    return I2nn;
}

Real LinkStewart::get_m1() const
//! @brief Return the mass of part 1.
{
    return m1;
}

Real LinkStewart::get_m2() const
//! @brief Return the mass of part 2.
{
    return m2;
}

Real LinkStewart::get_Lenght1()  const
//! @brief Return the lenght between platform attachment point and center of mass of part 1. 
{
    return Lenght1;
}

Real LinkStewart::get_Lenght2() const
//! @brief Return the lenght between base attachment point and center of mass of part 2.
{
    return Lenght2;
}

void LinkStewart::LTransform(const Matrix wRp, const ColumnVector q)
/*!
  @brief Recalculate the link's parameters related to the platform position.
  @param wRp: rotation matrix.
  @param q: Position of the platform.
*/
{
    aPos = Find_a(wRp,q);
    L = Find_Lenght();
    UnitV = Find_UnitV();
    Vv = Find_VctV();
    Vc = Find_VctC();
}

void LinkStewart::d_LTransform(const ColumnVector dq, const ColumnVector Omega, 
			       const Real dl, const Real ddl)
/*!	
  @brief Recalculate the link's parameters related to the platform speed.
  @param dq: Speed of the platform.
  @param Omega: Agular speed of the platform.
  @param dl: Extension rate of the link.
  @param ddl: Extension acceleration of the link.
*/
{
    Matrix AngularKin;

    da = Find_da(dq,Omega);
    AngularKin = Find_AngularKin(dl, ddl);
    LOmega = AngularKin.Column(1);
    LAlpha = AngularKin.Column(2);
}

void LinkStewart::dd_LTransform(const ColumnVector ddq, const ColumnVector Omega, 
				const ColumnVector Alpha, const Real dl, const Real ddl)
/*!
  @brief Recalculate the link's parameters related to the platform acceleration.
  @param ddq: Acceleration of the platform.
  @param Omega: Angular speed of the platform.
  @param Alpha: Angular acceleration of the platform.
  @param dl: Extension rate of the link.
  @param ddl: Extension acceleration of the link.
*/
{
    Matrix AngularKin;
    
    dda = Find_dda(ddq,Omega,Alpha);
    AngularKin = Find_AngularKin(dl, ddl);
    LOmega = AngularKin.Column(1);
    LAlpha = AngularKin.Column(2);
}

void LinkStewart::tau_LTransform(const Real dl, const Real ddl,const Real Gravity)
/*!
  @brief Recalculate the link's parameters related to the platform dynamics.
  @param dl: Extension rate of the link.
  @param ddl: Extension acceleration of the link.
  @param Gravity: Gravity (9.81).
*/
{
    ACM1 = Find_ACM1(dl,ddl);
    N = Find_N(Gravity);
}

ReturnMatrix LinkStewart::Find_a(const Matrix wRp, const ColumnVector q)
/*!
  @brief Return the position of the attachment point on the platform.
  @param wRp: Rotation matrix.
  @param q: Position of the platform.

  The position of the attachment point on the platform is equal to the position of
  the center of the platform plus the position of the attach (in the local 
  referencial) multiplicated by the rotation
  matrix:
  
  \f$ a = (x,y,z)_q + wRp \cdot a_l \f$
  
  where:
  - \f$a_l\f$ is the position of the attach in the local referencial
  - \f$(x,y,z)_q\f$ is the position of the platform center 
     (first 3 elements of the q vector)
*/
{
    ColumnVector a;
    a = q.Rows(1,3) + wRp*ap;
    a.Release();  
    return a;
}

/*!
  @brief Return the unit vector of the link direction

  The unit vector representing the orientation of the link is equal to:
  
  \f$ n = \frac{a_w - b}{Lenght} \f$
  
  where:
  - A is the position of the attachment point on the platform (world referential).
  - B is the position of the attachment point on the base (world referential).
  - Lenght is the lenght of the link.

*/
ReturnMatrix LinkStewart::Find_UnitV()
{
    Matrix Tmp (1,3);    
    Tmp = (aPos - b)/L;
    Tmp.Release(); 
    return Tmp;
}

/*!
  @fn LinkStewart::Find_da(const ColumnVector dq, const ColumnVector Omega)
  @brief Return the speed of the attachment point of the link on the platform
  @param dq: Speed of the platform
  @param Omega: Angular speed of the platform

    This function represent the equation:
    \f$ \dot{a} = (\dot{x},\dot{y},\dot{z})_p + \omega \times a_w\f$

  Where:
  - \f$(\dot{x},\dot{y},\dot{z})_p\f$ is the speed of the platform center (first 3 elements of dq vector)
  - \f$\omega\f$ is the angular speed of the platform
  - \f$a_w\f$ is the position of the attachment point of the link to the platform in the world referential
*/
ReturnMatrix LinkStewart::Find_da(const ColumnVector dq, const ColumnVector Omega)
{
    ColumnVector da;
    
    da = dq.Rows(1,3) + CrossProduct(Omega,aPos);
    
    da.Release(); 
    return da;
}

/*!
  @fn LinkStewart::Find_dda(const ColumnVector ddq,const ColumnVector Omega,const ColumnVector Alpha)
  @brief Return the acceleration of the attachment point of the link on the platform.
  @param ddq: Acceleration of the platform.
  @param Omega: Angular speed of the platform.
  @param Alpha: Angular acceleration of the platform

  This function represent the equation:
    \f$ \ddot{a} = (\ddot{x},\ddot{y},\ddot{z})_p + \alpha \times a_w + \omega\times(\omega\times a_w)\f$

  Where:
  - \f$(\ddot{x},\ddot{y},\ddot{z})_p\f$ is the acceleration of the platform center (first 3 elements of ddq vector)
  - \f$\alpha\f$ is the angular acceleration of the platform
  - \f$\omega\f$ is the angular speed of the platform
  - \f$a_w\f$ is the position of the attachment point of the link to the platform in the world referential
*/
ReturnMatrix LinkStewart::Find_dda(const ColumnVector ddq,const ColumnVector Omega,const ColumnVector Alpha)
{
    ColumnVector dda;
    dda = ddq.Rows(1,3) + CrossProduct(Alpha,aPos) +
	CrossProduct(Omega,CrossProduct(Omega,aPos));
    
    dda.Release(); 
    return dda;
}

/*!
  @fn LinkStewart::Find_Lenght()
  @brief Return the lenght of the link

  \f$l = \sqrt{(a_w - b)\cdot(a_w - b)}\f$

  where:
  - \f$a_w\f$ is the position of the attachment point of the link to 
  the platform in the world referential
  - b is the attachment point of the link to the base

*/
Real LinkStewart::Find_Lenght()
{
    return sqrt(DotProduct((aPos - b),(aPos - b)));
}

/*!
  @fn LinkStewart::Find_VctU()
  @brief Return the unit vector of the universal joint along the first axis of the fixed revolute joint

  This vector is equal to the unitary projection of the link unit vector on the X-Z plane:

  \f$ u_x = \frac{n_x}{\sqrt{n_x^2+n_z^2}}\f$;\f$ u_y =0 \f$;\f$ u_z = \frac{n_z}{\sqrt{n_x^2+n_z^2}}\f$
  
  where:
  - \f$ u_x\f$, \f$u_y\f$ and \f$u_z\f$ are the elements of the vector
  - \f$ n_x\f$ and \f$n_z\f$ are the x component and the z component of the link unit vector
*/
ReturnMatrix LinkStewart::Find_VctU()
{
    ColumnVector u(3);
    
    u(1) = -UnitV(1)/sqrt(UnitV(1)*UnitV(1) + UnitV(3)*UnitV(3));
    u(2) = 0.0;
    u(3) = -UnitV(3)/sqrt(UnitV(1)*UnitV(1) + UnitV(3)*UnitV(3));
    
    u.Release();
    return u;
}

/*!
  @fn LinkStewart::Find_VctV()
  @brief Return the unit vector of the universal joint along the second axis of the fixed revolute joint

  Eq:

  \f$ v = \frac{u\times n}{\Vert u \times n \Vert}\f$

  Where:
  - u is the unit vector of the universal joint along the first axis of the fixed revolute joint
  - n is the unit vector of the link
*/
ReturnMatrix LinkStewart::Find_VctV()
{
    ColumnVector v;
    v = CrossProduct(Vu,UnitV)/(CrossProduct(Vu,UnitV).NormFrobenius());    

    v.Release(); 
    return v;
}

/*!
  @fn LinkStewart::Find_VctC()
  @brief Return the unit vector of the universal joint along the third axis of the fixed revolute joint

  Eq:

  \f$ c= u \times v \f$

  Where:
  - u is the unit vector of the universal joint along the first axis of the fixed revolute joint
  - v is the unit vector of the universal joint along the second axis of the fixed revolute joint
*/
ReturnMatrix LinkStewart::Find_VctC()
{
    return CrossProduct(Vu, Vv);
}

/*!
  @fn LinkStewart::Find_AngularKin(const Real dl, const Real ddl)
  @brief Return the angular speed (Column 1) and angular acceleration (Column 2) of the link
  @param dl: Extention rate of the link
  @param ddl: Extention acceleration of the link

  Eqs for angular speed:

  \f$ \omega_u = -(\dot{a} - \dot{l}n)\cdot{v}/(ln\cdot{c})\f$

  \f$ \omega_v = (\dot{a}-\dot{l}n)\cdot{u}/(ln\cdot{c})\f$

  \f$ \omega = \omega_u u + \omega_v v\f$

  Eqs for angular acceleration:

  \f$\ddot{a}\prime\ = \ddot{a}-\omega_u\omega_v lc \times n - \ddot{l}n - 2\dot{l}\omega\times n - l\omega\times(\omega\times n)\f$

  \f$ \alpha_u = -\ddot{a}\prime\cdot{v}/(ln\cdot{c})\f$

  \f$ \alpha_v = \ddot{a}\prime\cdot{u}/(ln\cdot{c})\f$

  \f$ \alpha = \alpha_u u + \alpha_v v +\omega_u\omega_vc\f$

  where:
  - \f$ \dot{a}\f$ is the speed of the attachment point of the link to the platform
  - \f$\dot{l}\f$ is the extension rate of the link
  - n is the unit vector of the link
  - l is the lenght of the link
  - u, v, c are the rot. vectors of the universal joint
*/

ReturnMatrix LinkStewart::Find_AngularKin(const Real dl, const Real ddl)
{
    Matrix Temp(3,2);
    ColumnVector tmp_dda(3), omega(3), alpha(3);
    Real wu, wv, au, av;
    
    wu = DotProduct(-(da-dl*UnitV),Vv/(DotProduct(L*UnitV,Vc)));
    wv = DotProduct((da-dl*UnitV),Vu/(DotProduct(L*UnitV,Vc)));
    
    omega = wu*Vu + wv*Vv;
    
    tmp_dda = dda - wu*wv*L*CrossProduct(Vc,UnitV)-ddl*UnitV-2*dl*CrossProduct(omega,UnitV)
	-L*CrossProduct(omega,CrossProduct(omega,UnitV));
    
    au = DotProduct(-tmp_dda,Vv/(L*DotProduct(UnitV,Vc)));
    av = DotProduct(tmp_dda,Vu/(L*DotProduct(UnitV,Vc)));
    
    alpha = au*Vu + av*Vv + wu*wv*Vc;
    
    Temp.Column(1) = omega; Temp.Column(2) = alpha;
    
    Temp.Release(); return Temp;
}

/*!
  @brief Return the intermediate matrix N for force calculation 
  @param Gravity: Gravity (9.81)

  Eqs:
  
  \f$ I_1 = I_{1aa}nn^{T} + I_{1nn}(I_{3\times3}-nn^T)\f$
  
  \f$ I_2 = I_{2aa}nn^{T} + I_{2nn}(I_{3\times3}-nn^T)\f$
  
  \f[ N = -m_1(l-l_1)n\times{G}-m_2l_2(n\times{G})+(I_1+I_2)\alpha-(I_1+I_2)\omega\times{\omega}+m_1(l-l_1)n\times{a_1}+m_2l_2n\times{a_2}\f]
  
  Eq for \f$a_2\f$ (\f$a_1\f$ is found with the Find_ACM1 function):
  
  \f$ a_2 = l_2\omega\times{(\omega\times{n})}+l_2\alpha\times{n} \f$
	
  Where:
  - \f$I_{1aa}\f$ and \f$I_{2aa}\f$ are the mass moment of inertia component about the main axis of the two parts of the link
  - \f$I_{1nn}\f$ and \f$I_{2nn}\f$ are the mass moment of inertia component about the normal axis of the two parts of the link
  - n is the unit vector of the link
  - \f$I_{3\times3}\f$	is a identity matrix
  - \f$m_1\f$ is the mass of the first part of the link
  - l is the lenght of the link
  - \f$l_1\f$ is the distance between the center of mass of the first part of the link and the base
  - G is the gravity
  - \f$m_2\f$ is the mass of the second part of the link
  - \f$l_2\f$ is the distance between the center of mass of the second part of the link and the platform
  - \f$\alpha\f$ is the angular acceleration of the link
  - \f$\omega\f$ is the angular speed of the link
  - \f$a_1\f$ and \f$a_2\f$ are the acceleration of the center of mass of the two parts of the links
*/

ReturnMatrix LinkStewart::Find_N(const Real Gravity)
{
    ColumnVector  Accel2(3);
    Matrix Fn(3,1), N_N(3,1), I1(3,3), I2(3,3);
    
    gravity = 0;
    gravity(2) = -Gravity;

    Accel2 = Lenght2*CrossProduct(LOmega,CrossProduct(LOmega, UnitV)) 
             + Lenght2*CrossProduct(LAlpha,UnitV);
    
    IdentityMatrix Identity(3);
    
    I1 = I1aa*UnitV*UnitV.t() + I1nn*(Identity - UnitV*UnitV.t());
    I2 = I2aa*UnitV*UnitV.t() + I2nn*(Identity - UnitV*UnitV.t());
    
    N_N = -m1*(L-Lenght1)*CrossProduct(UnitV, gravity) - m2*Lenght2*CrossProduct(UnitV, gravity) +
	(I1+I2)*LAlpha - (I1+I2)*CrossProduct(LOmega,LOmega) 
	+ m1*(L-Lenght1)*CrossProduct(UnitV,ACM1)
	+m2*Lenght2*CrossProduct(UnitV,Accel2);
    
    N_N.Release(); 
    return N_N;
}

ReturnMatrix LinkStewart::Moment()
/*!
  @brief Return the moment component transmitted to the link from 
         the base or the platform (depending where is the universal joint)
  
  Eq:
  
  \f$ M = N\cdot{n}/c\cdot{n} \f$
  
  Where:
  - N is an intermediate matrix (Find_N)
  - n is the unit vector of the link
  - c is the rot. vector along the normal axis of the universal joint
*/
{
    Matrix M(3,1);    
    M = DotProduct(N, UnitV)/DotProduct(Vc, UnitV);
    M.Release(); 
    return M;	
}

ReturnMatrix LinkStewart::NormalForce()
/*!
  @brief Return the normal component of the reaction force of the platform 
         acting on the link
	
  Eq:
  
  \f$ F^n = (N\times{n} - M\times{n})/l \f$
  
  Where:
  - N is an intermediate matrix (Find_N())
  - n is the unit vector of the link
  - M is the reaction moment on the link (Moment())
  - l is the lenght of the link
*/
{
    Matrix Fn;
    Fn = (CrossProduct(N, UnitV) - CrossProduct(Moment(), UnitV))/L;    
    Fn.Release(); 
    return Fn;
}

/*!
  @fn LinkStewart::AxialForce(const Matrix J1, const ColumnVector C, const int Index)
  @brief Return the axial component of the reaction force of the platform acting on the link
  @param J1: First intermidiate jacobian matrix (find with Stewart::Find_InvJacob1())
  @param C: Intermidiate matrix in the dynamics calculation (find with Stewart::Find_C())
  @param Index: Number of the link (1 to 6)
  
  Eq:
  
  \f$ \left(  \begin{array}{c}
  f_1^a\\
  \vdots\\
  f_6^a\end{array} \right) = J_1^TC\f$
  
  Where:
  - \f$J_q^T\f$ is the jacobian matrix
  - C is an intermediate matrix (Stewart::Find_C())
*/
ReturnMatrix LinkStewart::AxialForce(const Matrix J1, const ColumnVector C, 
				     const int Index)
{
    Matrix Fa;
    ColumnVector tmp;
    tmp = (J1.t()*C);
    Fa = tmp(Index)*UnitV;
    
    Fa.Release(); 
    return Fa;
}

/*!
  @fn LinkStewart::ActuationForce(const Matrix J1, const ColumnVector C, 
                  const int Index, const Real Gravity)
  @brief Return the actuation force that power the prismatic joint
  @param J1: First intermidiate jacobian matrix (find with Stewart::Find_InvJacob1())
  @param C: Intermidiate matrix in the dynamics calculation (find with Stewart::Find_C())
  @param Index: Number of the link (1 to 6)
  @param Gravity: Gravity (9.81)

  Eq:

  \f$ f =m_1a_1\cdot{n}-f^a - m_1G\cdot{n} \f$
  
  Where:
  - m_1 is the mass of the first part of the link
  - a_1 is the acceleration of the center of mass of the first part of the link
  - n is the unit vector of the link
  - \f$f^a\f$ is from LinkStewart::AxialForce
  - G is gravity
*/
Real LinkStewart::ActuationForce(const Matrix J1, const ColumnVector C, 
				 const int Index, const Real Gravity)
{
    Real f, fa;

    ColumnVector Fa(3);
    gravity = 0;
    gravity(2) = -Gravity;
    
    Fa = AxialForce(J1, C, Index);
    
    if (Fa(1) != 0)
	fa = Fa(1)/UnitV(1);
    else if (Fa(2) != 0)
	fa = Fa(2)/UnitV(2);
    else if (Fa(3) != 0)
	fa = Fa(3)/UnitV(3);
    else
	fa = 0;
    
    f = m1*DotProduct(ACM1,UnitV) - fa - m1*DotProduct(gravity, UnitV);
    
    return f;
}
/*!
  @fn LinkStewart::Find_ACM1(const Real dl, const Real ddl)
  @brief Return the acceleration of the center of mass of the first part of the link 
  @param dl: Extention rate of the link
  @param ddl: Extention acceleration of the link

  Eq:
  
  \f$ a_1 = (l-l_1)\omega\times{(\omega\times{n})}+(l-l_1)\alpha\times{n}+2\omega\times{\dot{l}n}+\ddot{l}n\f$

  Where:
  - l is the lenght of the link
  - \f$l_1\f$ is the distance between the center of mass of the first part of the link to the base
  - \f$\omega\f$ is the angular speed of the link
  - \f$\alpha\f$ is the angular acceleration of the link
  - n is the unit vector of the link
  - \f$\dot{l}\f$ is the extension rate of the link
  - \f$\ddot{l}\f$ is the extension acceleration of the link
*/
ReturnMatrix LinkStewart::Find_ACM1(const Real dl, const Real ddl)
{
   ColumnVector Accel1;
    
    Accel1 = (L-Lenght1)*CrossProduct(LOmega, CrossProduct(LOmega, UnitV)) +
	(L-Lenght1)*CrossProduct(LAlpha,UnitV) +
	CrossProduct(2*LOmega,dl*UnitV)+ddl*UnitV;
    
    Accel1.Release(); 
    return Accel1;
}


/*!
  @fn Stewart::Stewart()
  @brief Default Constructor.
*/
Stewart::Stewart()
{
    q = ColumnVector(6);
    q = 0.0;
    dq = ColumnVector(6);
    dq = 0.0;
    ddq = ColumnVector(6);
    ddq = 0.0;
    pR = ColumnVector(3);
    pR = 0.0;
    gravity = pR;
    pIp = Matrix(3,3);
    pIp = 0.0;
    mp = 0.0;
    p = 0.0;
    n = 0.0;
    Js = 0.0;
    Jm = 0.0;
    bs = 0.0;
    bm = 0.0;
    p = 0.0;
    n = 0.0;
    Kb = 0.0;
    L = 0.0;
    R = 0.0;
    Kt = 0.0;
}

/*!
  @fn Stewart::Stewart(const Matrix InitPlatt, bool Joint)
  @brief Constructor.
  @param InitPlatt: Platform initialization matrix.
  @param Joint: bool indicating where is the universal joint
*/
Stewart::Stewart(const Matrix InitPlatt, bool Joint)
{
    ColumnVector InitLink (14);
    Matrix Inertia (3,3);
    
    pR = InitPlatt.SubMatrix(7,7,1,3).t();
    
    Inertia = 0.0;
    Inertia(1,1) = InitPlatt(7,4);
    Inertia(2,2) = InitPlatt(7,5);
    Inertia(3,3) = InitPlatt(7,6);
    pIp = Inertia;
    mp = InitPlatt(7,7);
    Js = InitPlatt(7,8);
    Jm = InitPlatt(7,9);
    bs = InitPlatt(7,10);
    bm = InitPlatt(7,11);
    p = InitPlatt(7,12);
    n = InitPlatt(7,13);
    Kb = InitPlatt(7,14);
    L = InitPlatt(7,15);
    R = InitPlatt(7,16);
    Kt = InitPlatt(7,17);
    
    UJointAtBase = Joint;
    
    q = ColumnVector(6); q <<0.0 <<0.0 <<1.0 <<0.0 <<M_PI/2 <<0.0;
    dq = ColumnVector(6); dq =0.0;
    ddq = ColumnVector(6); ddq =0.0;

    wRp = Find_wRp();

    gravity = ColumnVector(3);
    gravity = 0;
    
    for (int i =0; i<6; i++)
    {
	InitLink = InitPlatt.Row(i+1).t();
	Links[i] = LinkStewart (InitLink,wRp,q);
    }
}

/*!
	@fn Stewart::Stewart(const string & FileName, const string & PlatFormName)
	@brief Constructor from a .conf file.
	@param FileName: Name of the .conf file
	@param PlatFormName: Name of the platform
*/
Stewart::Stewart(const string & FileName, const string & PlatFormName)
{
    pR = ColumnVector(3);
    pR = 0.0;
    gravity = pR;
    pIp = Matrix(3,3);
    pIp = 0.0;
    mp = 0.0;
    
    ColumnVector InitLink(14);
    
    Config platData;
    ifstream inconffile(FileName.c_str(), std::ios::in);
    if (platData.read_conf(inconffile))
    {
        cerr << "Stewart::Stewart: can not read input config file" << endl;
    }
    
    platData.select(PlatFormName, "mp", mp);
    platData.select(PlatFormName, "Joint", UJointAtBase);
    platData.select(PlatFormName, "pRx", pR(1));
    platData.select(PlatFormName, "pRy", pR(2));
    platData.select(PlatFormName, "pRz", pR(3));
    platData.select(PlatFormName, "Ixx", pIp(1,1));
    platData.select(PlatFormName, "Iyy", pIp(2,2));
    platData.select(PlatFormName, "Izz", pIp(3,3));
    platData.select(PlatFormName, "Js", Js);
    platData.select(PlatFormName, "Jm", Jm);
    platData.select(PlatFormName, "bs", bs);
    platData.select(PlatFormName, "bm", bm);
    platData.select(PlatFormName, "p", p);
    platData.select(PlatFormName, "n", n);
    platData.select(PlatFormName, "Kb", Kb);
    platData.select(PlatFormName, "L", L);
    platData.select(PlatFormName, "R", R);
    platData.select(PlatFormName, "Kt", Kt);

    q = ColumnVector(6); q <<0.0 <<0.0 <<1.0 <<0.0 <<M_PI/2 <<0.0;
    dq = ColumnVector(6); dq =0.0;
    ddq = ColumnVector(6); ddq =0.0;
    
    wRp = Find_wRp();
    
    for(int j = 1; j <= 6; j++)
    {
	string platformName_link;
	ostringstream ostr;
	ostr << PlatFormName << "_LINK" << j;
	platformName_link = ostr.str();
	
	platData.select(platformName_link, "bx", InitLink(1));
	platData.select(platformName_link, "by", InitLink(2));
	platData.select(platformName_link, "bz", InitLink(3));
	platData.select(platformName_link, "ax", InitLink(4));
	platData.select(platformName_link, "ay", InitLink(5));
	platData.select(platformName_link, "az", InitLink(6));
	platData.select(platformName_link, "Iaa1", InitLink(7));
	platData.select(platformName_link, "Inn1", InitLink(8));
	platData.select(platformName_link, "Iaa2", InitLink(9));
	platData.select(platformName_link, "Inn2", InitLink(10));
	platData.select(platformName_link, "m1", InitLink(11));
	platData.select(platformName_link, "m2", InitLink(12));
	platData.select(platformName_link, "L1", InitLink(13));
	platData.select(platformName_link, "L2", InitLink(14));

	Links[j-1] = LinkStewart(InitLink,wRp,q);	
    }
}

/*!
	@fn Stewart::Stewart(const Stewart & x)
	@brief Copy Constructor
*/
Stewart::Stewart(const Stewart & x)
{
    for(int i=0;i<6;i++)
	Links[i] = x.Links[i];
    UJointAtBase = x.UJointAtBase;
    q = x.q;
    dq = x.dq;
    ddq = x.ddq;
    pR = x.pR;
    pIp = x.pIp;
    mp = x.mp;
    p = x.p;
    n = x.n;
    Js = x.Js;
    Jm = x.Jm;
    bs = x.bs;
    bm = x.bm;
    Kb = x.Kb;
    L = x.L;
    R = x.R;
    Kt = x.Kt;
    gravity = x.gravity;
}


Stewart::~Stewart()
//! @brief Destructor.
{	
}

const Stewart & Stewart::operator = (const Stewart& x)
{
    //! @brief Overload = operator.
    for (int i=0; i<6; i++)
	Links[i] = x.Links[i];
    UJointAtBase = x.UJointAtBase;
    q = x.q;
    dq = x.dq;
    ddq = x.ddq;
    pR = x.pR;
    pIp = x.pIp;
    mp = x.mp;
    p = x.p;
    n = x.n;
    Js = x.Js;
    Jm = x.Jm;
    bs = x.bs;
    bm = x.bm;
    Kb = x.Kb;
    L = x.L;
    R = x.R;
    Kt = x.Kt;
    gravity = x.gravity;
    return *this;
}

void Stewart::set_Joint(const bool Joint) 
//! @brief Set the position of the universal joint on the links
{
    UJointAtBase = Joint;
}

void Stewart::set_q(const ColumnVector _q)
//! @brief Set the position of the platform.
{
    if(_q.Nrows()== 6)
    {
	q = _q;
	Transform();
    }
    else
	cerr<< "Stewart::set_q: wrong size in input vector."<< endl;
}

void Stewart::set_dq(const ColumnVector dq_)
//! @brief Set the platform's speed.
{
    if(dq_.Nrows()== 6)
    {
	dq = dq_;
	
	Omega = Find_Omega();
	dl = Find_dl();
	ddl =  Find_ddl();
	
	for(int i =0; i<6; i++)
	    Links[i].d_LTransform(dq,Omega, dl(i+1),ddl(i+1));
    }
    else
	cerr<< "Stewart::set_dq: wrong size in input vector."<< endl;	
}

void Stewart::set_ddq(const ColumnVector _ddq)
//! @brief Set the platform's acceleration.
{
    if(_ddq.Nrows()== 6)
    {
	ddq = _ddq;
	
	Omega = Find_Omega();
	Alpha = Find_Alpha();
	ddl = Find_ddl();
	for(int i =0; i<6; i++)
	    Links[i].dd_LTransform(ddq,Omega,Alpha, dl(i+1),ddl(i+1));	
    }
    else
	cerr<< "Stewart::set_ddq: wrong size in input vector."<< endl;
}

void Stewart::set_pR(const ColumnVector _pR)
//! @brief Set the position of the center of mass of the platform
{
    if(_pR.Nrows()== 3)
	pR = _pR;
    else
	cerr<< "Stewart::set_pR: wrong size in input vector."<< endl;
}

void Stewart::set_pIp(const Matrix _pIp)
//! @brief Set the inertia matrix of the platform
{
    if((_pIp.Nrows()== 3)&&(_pIp.Ncols() == 3))
	pIp = _pIp;
    else
	cerr<< "Stewart::set_pIp: wrong size in input vector."<< endl;
}

void Stewart::set_mp (const Real _mp)
//! @brief Set the mass of the platform
{
    mp = _mp;
} 
    
bool Stewart::get_Joint () const
//! @brief Return the position of the universal joint (true if at base, false if at platform)
{
    return UJointAtBase;
}

ReturnMatrix Stewart::get_q () const
//! @brief Return the position of the platform
{
    return q;
}

ReturnMatrix Stewart::get_dq () const
//! @brief Return the speed of the platform
{
    return dq;
}

ReturnMatrix Stewart::get_ddq () const
//! @brief Return the acceleration of the platform
{
    return ddq;
}

ReturnMatrix Stewart::get_pR () const
//! @brief Return the postion of the center of mass of the platfom
{
    return pR;
}

ReturnMatrix Stewart::get_pIp () const
//! @brief Return the inertia matrix of the platform
{
    return pIp;
}

Real Stewart::get_mp() const
//! @brief Return the mass of the platform
{
    return mp;
}

/*!
  @fn Stewart::Transform()
  @brief Call the functions corresponding to the basic parameters when q changes
  
  These functions are called by Transform:
  - Find_wRp()
  - LinkStewart::LTransform() for each link
  - Find_InvJacob1()
  - Find_InvJacob2()
  - jacobian()
*/
void Stewart::Transform()
{
    wRp = Find_wRp();
    
    for(int i=0; i<6; i++)
	Links[i].LTransform(wRp, q);
    
    IJ1 = Find_InvJacob1();
    IJ2 = Find_InvJacob2();
    Jacobian = jacobian();	
}

/*!
  @fn Stewart::Find_wRp ()
  @brief Return the rotation matrix wRp.
  
  Eq of the matrix:
  
  \f$ wRp =  \left(  \begin{array}{ccc}
  \cos(\psi)\cos(\phi)-\cos(\theta)\sin(\phi)\sin(\psi) & -\sin(\psi)\cos(\phi)-\cos(\theta)\sin(\phi)\cos(\psi) & \sin(\theta)\sin(\phi) \\
  \cos(\psi)\sin(\phi)+\cos(\theta)\cos(\phi)\sin(\psi) & -\sin(\psi)\sin(\phi)+\cos(\theta)\cos(\phi)\cos(\psi) & -\sin(\theta)\cos(\phi) \\
  \sin(\psi)\sin(\theta) & \cos(\psi)\sin(\theta) & \cos(\theta) \end{array} \right)\f$
  
  Where:
  - \f$\psi,\theta, \phi,\f$ are the three Euler angles of the platform.
*/
ReturnMatrix Stewart::Find_wRp ()
{
    Matrix _wRp(3,3);
    
    _wRp(1,1) = cos(q(6))*cos(q(4)) - cos(q(5))*sin(q(4))*sin(q(6)); 
    _wRp(1,2) = -sin(q(6))*cos(q(4)) - cos(q(5))*sin(q(4))*cos(q(6));
    _wRp(1,3) = sin(q(5))*sin(q(4));
    _wRp(2,1) = sin(q(6))*cos(q(4)) + cos(q(5))*sin(q(4))*cos(q(6));
    _wRp(2,2) = -sin(q(6))*sin(q(4)) + cos(q(6))*cos(q(4))*cos(q(5));
    _wRp(2,3) = -sin(q(5))*cos(q(4));
    _wRp(3,1) = sin(q(6))*sin(q(5));
    _wRp(3,2) = sin(q(5))*cos(q(6));
    _wRp(3,3) = cos(q(5));
    
    _wRp.Release(); 
    return _wRp;
}

/*!
  @fn Stewart::Find_Omega()
  @brief Return the angular speed of the platform

  Eq:
  
  \f$ \omega =  \left(  \begin{array}{ccc}
  0 & \cos(\phi) & \cos(\theta)\sin(\phi) \\
  0 & \sin(\phi) & -\sin(\theta)\cos(\phi) \\
  1 & 0 & \cos(\theta) \end{array} \right) \left(  \begin{array}{c}
  \dot{\phi}\\
  \dot{\theta}\\
  \dot{\psi}\end{array} \right)\f$

  Where:
  - \f$\psi,\theta, \phi\f$ are the three Euler angles of the platform.
  - \f$\dot{\psi},\dot{\theta},\dot{\phi}\f$ are the three Euler angle speed of the platform.
	
*/
ReturnMatrix Stewart::Find_Omega()
{
    ColumnVector w(3);
    
    w(1) = cos(q(4))*dq(5) + sin(q(4))*cos(q(5))*dq(6);
    w(2) = sin(q(4))*dq(5) - cos(q(4))*sin(q(5))*dq(6);
    w(3) = dq(4) + cos(q(5))*dq(6);
    
    w.Release(); 
    return w;
}

/*!
  @fn Stewart::Find_Alpha()
  @brief Return the angular acceleration of the platform

  Eq:
  
  \f$ \alpha =  \left(  \begin{array}{ccc}
  0 & \cos(\phi) & \cos(\theta)\sin(\phi) \\
  0 & \sin(\phi) & -\sin(\theta)\cos(\phi) \\
  1 & 0 & \cos(\theta) \end{array} \right) \left(  \begin{array}{c}
  \ddot{\phi}\\
  \ddot{\theta}\\
  \ddot{\psi}\end{array} \right)
  + \left(  \begin{array}{ccc}
  0 & -\phi\sin(\phi) & \phi\cos(\phi)\sin(\theta) + \dot{\theta}\sin(\phi)\cos(\theta)\\
  0 & \phi\cos(\phi) & \phi\sin(\phi)\sin(\theta) - \dot{\theta}\cos(\phi)\cos(\theta) \\
  0 & 0 & -\dot{theta}\sin(\theta)\end{array} \right) \left(  \begin{array}{c}
  \dot{\phi}\\
  \dot{\theta}\\
  \dot{\psi}\end{array} \right)\f$
  
  Where:
  - \f$\psi,\theta, \phi\f$ are the three Euler angles of the platform.
  - \f$\dot{\psi},\dot{\theta},\dot{\phi}\f$ are the three Euler angle speed of the platform.
  - \f$\ddot{\psi},\ddot{\theta},\ddot{\phi}\f$ are the three Euler angle acceleration of the platform.
*/
ReturnMatrix Stewart::Find_Alpha()
{
    Matrix A, Temp(3,3),Temp2(3,3);
    
    Temp = 0.0;	Temp(3,1) = 1; Temp(1,2) = cos(q(4)); Temp(2,2) = sin(q(4));
    Temp(1,3) = sin(q(4))*cos(q(5)); Temp(2,3)=-cos(q(4))*sin(q(5)); Temp(3,3) = cos(q(5));
    
    Temp2 = 0.0; Temp2(1,2) = -dq(4)*sin(q(4)); Temp2(2,2) = dq(4)*cos(q(4));
    Temp2(1,3) = dq(4)*cos(q(4))*sin(q(5))+dq(5)*sin(q(4))*cos(q(5));
    Temp2(2,3) = dq(4)*sin(q(4))*sin(q(5))-dq(5)*cos(q(4))*cos(q(5));
    Temp2(3,3) = -dq(5)*sin(q(5));
    
    A = Temp*ddq.Rows(4,6) + Temp2*dq.Rows(4,6);
    
    A.Release(); 
    return A;
}

/*!
  @fn Stewart::jacobian()
  @brief Return the jacobian matrix of the platform

  Eq:
  
  \f$J = J_1^{-1}J_2^{-1}\f$
  
  Where:
  - \f$J_1\f$ and \f$J_2\f$ are intermediate matrix(Find_InvJacob1(), Find_InvJacob2())
*/
ReturnMatrix Stewart::jacobian()
{
    Matrix _Jacobi;
    
    _Jacobi = (IJ1*IJ2).i();
    
    _Jacobi.Release(); 
    return _Jacobi;
}

/*!
  @fn Stewart::Find_InvJacob1()
  @brief Return the first intermediate jacobian matrix (reverse) of the platform
  
  Eq:
  
  \f$ J_1^{-1} = \left(  \begin{array}{cc}
  n_1^T & (a_{w1}\times{n_1})^T\\
  \vdots & \vdots\\
  n_6^T & (a_{w6}\times{n_6})^T\end{array} \right)\f$
  
  Where:
  - \f$ n_1\f$ to \f$ n_6 \f$ are the unit vector of the links
  - \f$ a_{w1}\f$ to \f$a_{w6}\f$ are the attachment point of the links to the platform 
     in the world referential
*/
ReturnMatrix Stewart::Find_InvJacob1()
{
    Matrix tmp_Jacobi1 (6,6);
    
    for(int i = 0; i<6; i++)
	tmp_Jacobi1.Row(i+1) = Links[i].UnitV.t() | CrossProduct(wRp*Links[i].ap,Links[i].UnitV).t();
    
    tmp_Jacobi1.Release(); 
    return tmp_Jacobi1;
}

/*!
  @fn Stewart::Find_InvJacob2()
  @brief Return the second intermediate jacobian matrix (reverse) of the platform
  
  Eq:
  
  \f$ J_2^{-1} = \left(  \begin{array}{cccccc}
  1&0&0&0&0&0\\
  0&1&0&0&0&0\\
  0&0&1&0&0&0\\
  0&0&0&0&\cos\phi&\sin\phi\sin\theta\\
  0&0&0&0&\sin\phi&-\cos\phi\sin\theta\\
  0&0&0&1&0&\cos\theta\end{array} \right)\f$
  
  Where:
  - \f$\phi\f$ and \f$\theta\f$ are two of the euler angle of the platform (vector q)
*/
ReturnMatrix Stewart::Find_InvJacob2()
{
    Matrix tmp_Jacobi2;

    tmp_Jacobi2 = IdentityMatrix(6);
    tmp_Jacobi2(4,4) = 0;
    tmp_Jacobi2(6,4) = 1;
    tmp_Jacobi2(4,5) = cos(q(4));
    tmp_Jacobi2(5,5) = sin(q(4));
    tmp_Jacobi2(4,6) = sin(q(4))*sin(q(5));
    tmp_Jacobi2(5,6) = -cos(q(4))*sin(q(5));
    tmp_Jacobi2(6,6) = cos(q(5));

    tmp_Jacobi2.Release(); 
    return tmp_Jacobi2;
}
/*!
  @fn Stewart::jacobian_dot()
  @brief Return time deriative of the inverse jacobian matrix of the platform
  
  Eq:
  
  \f$ \frac{dJ^{-1}}{dt} = \frac{dJ_1^{-1}}{dt}J_2^{-1}+J_1^{-1}\frac{dJ_2^{-1}}{dt}\f$
  
  \f$ \frac{dJ_1^{-1}}{dt} = \left(  \begin{array}{cc}
  (\omega_1\times{n_1})^T & ((\omega\times{a_{w1}})\times{n_1}+a_{w1}\times{(\omega_1\times{n_1})})^T\\
  \vdots & \vdots\\
  (\omega_6\times{n_6})^T & ((\omega\times{a_{w6}})\times{n_6}+a_{w6}\times{(\omega_6\times{n_6})})^T\end{array} \right)\f$
  
  \f$ \frac{dJ_2^{-1}}{dt} = \left(  \begin{array}{cccccc}
  0 & 0 & 0 & 0 & 0 & 0\\
  0 & 0 & 0 & 0 & 0 & 0\\
  0 & 0 & 0 & 0 & 0 & 0\\
  0 & 0 & 0 & 0 & -\dot{\phi}\sin\phi & \dot{\phi}\cos\phi\sin\theta + \dot{\theta}\sin\phi\cos\theta\\
  0 & 0 & 0 & 0 & \dot{\phi}\cos\phi & \dot{\phi}\sin\phi\sin\theta + \dot{\theta}\cos\phi\cos\theta\\
  0 & 0 & 0 & 0 & 0 &-\dot{\theta}\sin\theta\end{array} \right)\f$

  Where:
  - \f$\omega_i\f$ is the angular speed vector of each link
  - n is the unit vector of the link
  - \f$\omega\f$ is the angular speed vector of the platform
  - \f$a_{wi}\f$ is the position vector of the attachment point of the link to the platform
  - \f$\phi\f$ and \f$\theta\f$ are two of the Euler angle (vector q)
  - \f$\dot{\phi}\f$ and \f$\dot{\theta}\f$ are two of the Euler angle speed (vector dq)
*/
ReturnMatrix Stewart::jacobian_dot()
{
    Matrix tmp_dJ2(6,6), tmp_dn(6,3), tmp_sol(6,3), tmp_dJ1(6,6), tmp_dJ(6,6);
    ColumnVector VctNorm, a;

    tmp_dJ2 = 0.0;
    tmp_dJ2(4,5) = -dq(4)*sin(q(4));
    tmp_dJ2(5,5) = dq(4)*cos(q(4));
    tmp_dJ2(4,6) = dq(4)*cos(q(4))*sin(q(5))+dq(5)*sin(q(4))*cos(q(5));
    tmp_dJ2(5,6) = dq(4)*sin(q(4))*sin(q(5))-dq(5)*cos(q(4))*cos(q(5));
    tmp_dJ2(6,6) = -dq(5)*sin(q(5));
    
    for (int i = 0; i<6; i++)
    {
	tmp_dn.Row(i+1) = ((Links[i].UnitV*Links[i].L - 
			    dl(i+1)*Links[i].UnitV)/Links[i].L).t();
	tmp_sol.Row(i+1) = (CrossProduct(CrossProduct(Omega,Links[i].aPos.t()),
					 Links[i].UnitV.t()) +
			    (CrossProduct(Links[i].aPos,tmp_dn.Row(i+1)))).t();
    }
    
    for (int j = 1; j < 7; j++)
	for(int k = 1; k < 4; k++)
	{
	    tmp_dJ1(j,k) = tmp_dn(j,k);
	    tmp_dJ1(j,k+3) = tmp_sol(j,k);
	}
    
    tmp_dJ = tmp_dJ1*IJ2 + IJ1*tmp_dJ2;
    
    tmp_dJ.Release(); 
    return tmp_dJ;
}
/*!
  @fn Stewart::InvPosKine()
  @brief Return the lenght of the links in a vector
  
  The goal of the inverse kinematic is to find the lenght of each of the 
   six links from the position of the platform (X,Y,Z,\f$\psi\f$,\f$\theta\f$,\f$\phi\f$).
*/
ReturnMatrix Stewart::InvPosKine()
{
   ColumnVector Vct_L(6);

    for (int i = 1; i < 7; i++)
	Vct_L(i) = Links[i-1].L;
    
    Vct_L.Release(); return Vct_L;
}

/*!
  @fn Stewart::Find_dl()
  @brief Return the extension rate of the links in a vector
  
  Eq:
  
  \f$\dot{l} = J^{-1}\dot{q}\f$
  
  Where:
  - \f$J^{-1}\f$ is the inverse Jacobian matrix of the platform
  - \f$\dot{q}\f$ is the dq vector	 
*/
ReturnMatrix Stewart::Find_dl()
{
    ColumnVector tmp_dl;
    tmp_dl = Jacobian.i()*dq;

    tmp_dl.Release(); 
    return tmp_dl;
}

/*!
  @fn Stewart::Find_ddl()
  @brief Return the extension acceleration of the links in a vector
  
  Eq:
  
  \f$ \ddot{l} = J^{-1}\ddot{q} + \frac{dJ^{-1}}{dt}\dot{q}\f$
  
  Where:
  - \f$J^{-1}\f$ is the inverse jacobian matrix of the platform
  - \f$\ddot{q}\f$ is the ddq vector
*/
ReturnMatrix Stewart::Find_ddl()
{
    ColumnVector tmp_ddl;
    tmp_ddl = Jacobian.i() * ddq + jacobian_dot() * dq;    
    tmp_ddl.Release(); 
    return tmp_ddl;
}

/*!
  @fn Stewart::Find_C(const Real Gravity)
  @brief Return intermediate matrix C for the dynamics calculations
  
  Eqs:
  
  \f$\ddot{x}_g = \ddot{x}+\alpha\times{\bar{r}}+\omega(\omega\times{\bar{r}})\f$
  
  \f$ \bar{r} = ^{w}R_{p}\cdot{^{p}\bar{r}}\f$
  
  \f$ \bar{I}_p = ^wR_p^p\bar{I}_p^wR_p^T\f$
  
  \f$C = \left(  \begin{array}{c}
  m_pG - m_p\ddot{x}_g-\sum{F_i^n}\\
  m_p\bar{r}\times{G}-m_p(\bar{r}\times{\ddot{x}_g}-\bar{I}_p\alpha+\bar{I}_p\omega\times{\omega}-\sum{a_{wi}\times{F_i^n}}-\sum{M_i}\end{array} \right)\f$
  
  Where:
  - \f$\ddot{x}_g\f$ is the acceleration of the platform center of mass.
  - \f$\ddot{x}\f$ is the acceleration of the platform center (first three elements of the ddq vector).
  - \f$\alpha\f$ is the angular acceleration of the platform.
  - \f$\bar{r}\f$ is the platform center of mass in the world referential.
  - \f$\omega\f$ is the angular speed of the platform.
  - \f$^wR_p\f$ is the rotational matrix of the two referentials (world and platform).
  - \f$^{p}\bar{r}\f$ is the vector of the center of mass of the platform with reference to the local frame (platform).
  - \f$^p\bar{I}_p\f$ is the constant mass moments of inertia of the platform with reference to the local frame (platform).
  - \f$m_p\f$ is the mass of the platform.
  - G is the gravity.
  - \f$F_i^n\f$ is the normal force transferred from the platform to the link.
  - \f$\bar{I}_p\f$ is the constant mass moments of inertia of the platform in the world referential.
  - \f$a_{wi}\f$ is the position of the attachment point of each link to the platform in the world referential.
  - \f$M_i\f$ is the moment transferred from the platform to the link (not present is the spherical joint is at the platform end).
*/
ReturnMatrix Stewart::Find_C(const Real Gravity)
{
    Matrix C(6,1), I(3,3);
    ColumnVector Sum(3), Sum2(3), Sum3(3), ddxg(3), LNormalForce(3);

    gravity = 0;
    gravity(2) = -Gravity;
    
    ddxg = ddq.Rows(1,3) + CrossProduct(Alpha,wRp*pR) + CrossProduct(Omega,CrossProduct(Omega,wRp*pR));
    I = wRp*pIp*(wRp.t());
    
    Sum = 0.0;
    Sum2 = 0.0;
    Sum3 = 0.0;
    for (int i=0; i<6; i++)
    {	
	LNormalForce = Links[i].NormalForce();
	Sum = Sum + LNormalForce;
	Sum2 = Sum2 + CrossProduct(Links[i].aPos,LNormalForce);
	if(!UJointAtBase)
	{
	    Sum3 = Sum3 +Links[i].Moment();
	}
    }
    
    C.Rows(1,3) = mp*gravity - mp*ddxg - Sum;
    C.Rows(4,6) = mp*CrossProduct(wRp*pR, gravity) - mp*CrossProduct(wRp*pR,ddxg)-I*Alpha
	+ I*CrossProduct(Omega,Omega)- Sum2 - Sum3;
    
    C.Release(); 
    return C;	
}

/*!
  @fn Stewart::ForwardKine(const ColumnVector guess_q, const ColumnVector l_given, const Real tolerance)
  @brief Return the position vector of the platform (vector q)
  @param guess_q: Approximation of real position
  @param l_given: Lenght of the 6 links
  @param tolerance: Ending criterion

  The Newton-Raphson method is used to solve the forward kinematic problem.  It 
  is a numerical iterative technic that simplify the solution.  An approximation of 
  the answer has to be guess for this method to work.

  Eq:

  \f$ q_i = q_{i-1}-J_{q_{i-1}}(l_{q_{i-1}}-l)\f$
  
  Where:
  - \f$q_i\f$ is the position vector of the platform at the ith iteration.
  - \f$q_{i-1}\f$ is the position vector of the platform at the (i-1)th iteration.
  - \f$J_{q_{i-1}}\f$ is the Jacobian matrix of the platform at the position of the \f$q_{i-1}\f$ vector.
  - \f$l_{q_{i-1}}\f$ is the lenght vector of the links at the (i-1)th position of the platform.
  - l is the real lenght vector of the links.
*/
ReturnMatrix Stewart::ForwardKine(const ColumnVector guess_q, const ColumnVector l_given, 
				  const Real tolerance)
{
    ColumnVector next_q, tmp_long(6);
    Real Diff = 1;
    
    q = guess_q;
    while (Diff>tolerance)
    {
	for(int i=0; i<6; i++)
	    tmp_long(i+1) = Links[i].L - l_given(i+1);
	
	next_q = q - Jacobian*(tmp_long);
	Diff = (next_q - q).MaximumAbsoluteValue();
	
	set_q(next_q);
    }
    next_q.Release(); 
    return next_q;
}

/*!
  @fn Stewart::JointSpaceForceVct(const Real Gravity)
  @brief Return a vector containing the six actuation force components
  @param Gravity: Gravity (9.81)
  
  See the description of LinkStewart::ActuationForce().
*/
ReturnMatrix Stewart::JointSpaceForceVct(const Real Gravity)
{
    Matrix F(6,1), C, IJ1(6,6);

    IJ1 = Find_InvJacob1();
    C = Find_C(Gravity);
    
    for (int i =0; i<6; i++)
    {
	F(i+1,1) = Links[i].ActuationForce(IJ1, C, i+1, Gravity);
    }
    
    F.Release(); 
    return F;
}

/*!
  @fn Stewart::Torque(const Real Gravity)
  @brief Return the torque vector of the platform
  @param Gravity: Gravity (9.81)

  Eq:
  
  \f$\tau = J^{-T}F\f$
  
  Where:
  
  - \f$J\f$ is the Jacobian matrix of the platform.
  - F is the joint space force vector (JointSpaceForceVct()).
*/
ReturnMatrix Stewart::Torque(const Real Gravity)
{
    Matrix T;
    
    for(int i=0;i<6;i++)
	Links[i].tau_LTransform(dl(i+1), ddl(i+1), Gravity);
    
    T = Jacobian.i().t()*JointSpaceForceVct(Gravity);
    
    T.Release(); 
    return T;
}
/*!
  @fn Stewart::Find_h(const Real Gravity)
  @brief Return the intermediate matrix corresponding to the Coriolis and 
         centrifugal + gravity force/torque components 
  @param Gravity: Gravity (9.81)

  h is found by setting the ddq vector to zero and then calling the 
  torque routine.  The vector returned by Torque() is equal to h.
*/
ReturnMatrix Stewart::Find_h(const Real Gravity)
{
    ColumnVector _ddq(6);
    _ddq = 0.0;
    set_ddq(_ddq);
    return Torque(Gravity);
}

/*!
  @fn Stewart::Find_M()
  @brief Return the intermediate matrix corresponding to the inertia matrix of 
         the machine

  M is found by setting the dq and Gravity vectors to zero and the ddq vector to 
  zero except for the ith element that is set to one.  Then, the ith row of M is equal 
  to the matrix returned by Torque().
*/
ReturnMatrix Stewart::Find_M()
{
    Matrix M(6,6);
    ColumnVector _ddq(6), _dq(6), tmpdq(6);

    tmpdq = dq;
    _dq = 0.0;

    set_dq(_dq);
    
    for (int i = 1; i < 7; i++)
    {
	_ddq = 0.0;
	_ddq(i) = 1.0;
	set_ddq(_ddq);
	M.Column(i) = Torque (0.0);
    }
    set_dq(tmpdq);
	
    M.Release(); 
    return M;
}

/*!
  @fn Stewart::ForwardDyn(const ColumnVector T, const Real Gravity)
  @brief	Return the acceleration vector of the platform (ddq)
  @param T: torque vector
  @param Gravity: Gravity (9.81)
  
  Eq:
  
  \f$ ddq = M^{-1}(\tau-h)\f$
  
  Where:
  - M is from Find_M() routine.
  - \f$\tau\f$ is the torque vector.
  - h is from Find_h() routine.
*/
ReturnMatrix Stewart::ForwardDyn(const ColumnVector T, const Real Gravity)
{
    ColumnVector _ddq;
     
    _ddq = Find_M().i()*(T - Find_h(Gravity));
    
    _ddq.Release();  
    return _ddq;
}

/*!
  @fn Stewart::Find_Mc_Nc_Gc(Matrix & Mc, Matrix & Nc, Matrix & Gc)
  @brief	Return(!) the intermediates matrix for forward dynamics with actuator dynamics
  @param Mc: Inertia matrix of the machine
  @param Nc: Coriolis and centrifugal force/torque component
  @param Gc: Gravity force/torque component

  Eq:
  
  \f$K_a = \frac{p}{2\pi n}I_{6\times{6}}\f$
  \f$M_a = \frac{2\pi}{np}(J_s+n^2J_m)I_{6\times{6}}\f$
  \f$V_a = \frac{2\pi}{np}(b_s+n^2b_m)I_{6\times{6}}\f$
  \f$M_c = K_aJ^TM+M_aJ^{-1}\f$
  \f$N_c = K_aJ^TN+(V_aJ^{-1}+M_a\frac{dJ^{-1}}{dt})dq\f$  
  \f$G_c = K_aJ^TG\f$
  Where:
  - p is the pitch of the ballscrew.
  - n	is the gear ratio.
  - \f$I_{6\times{6}}\f$ is the Identity matrix.
  - \f$J_s\f$ is the mass moment of inertia of the ballscrew.
  - \f$J_m\f$ is the mass moment of inertia of the motor.
  - \f$b_s\f$ is the viscous damping coefficient of the ballscrew.
  - \f$b_m\f$ is the viscous damping coefficient of the motor.
  - J is the Jacobian matrix of the platform.
*/
void Stewart::Find_Mc_Nc_Gc(Matrix & Mc, Matrix & Nc, Matrix & Gc)
{
    Matrix G, Ka(6,6), Ma(6,6), Va(6,6), dJacobian(6,6);
    dJacobian = jacobian_dot();
    
    Ka = p/(2*M_PI*n)*IdentityMatrix(6);
    Ma = (2*M_PI/(n*p))*(Js + n*n*Jm)*IdentityMatrix(6);
    Va = (2*M_PI/(n*p))*(bs + n*n*bm)*IdentityMatrix(6);
    
    Mc = Ka*Jacobian.t() * Find_M() + Ma*Jacobian.i();
    
    Nc = Ka*Jacobian.t()*Find_h(0.0) + (Va*Jacobian.i()+Ma*dJacobian)*dq;
    
    ColumnVector _dq(6), _tmpdq(6);
    _dq = 0.0;
    _tmpdq = dq;
    set_dq(_dq);
    
    Gc = Ka *Jacobian.t()*Find_h();
    set_dq(_tmpdq);
}

/*!
  @fn Stewart::ForwardDyn_AD(const ColumnVector Command, const Real t)
  @brief Return the acceleration of the platform (Stewart platform mechanism dynamics 
         including actuator dynamics)
  @param Command: Vector of the 6 motors voltages.
  @param t: period of time use to find the currents (di/dt)
  
  Voltages with back emf:
  
  \f$V' = V-J^{-1}\dot{q}(\frac{2\pi}{p})K_b\f$
  
  Currents:
  
  \f$I = \frac{I_{6\times{6}}}{L}e^{(-R\cdot{t}/L)}V'\f$
  
  Motor torque:
  
  \f$\tau_m = IK_t\f$
  
  Platform acceleration:

  \f$\ddot{q} = M_c^{-1}(\tau_m - Nc - Gc)\f$
  
  Where:
  
  - J is the Jacobian matrix of the platform.
  - \f$\dot{q}\f$ is the dq vector.
  - p is the pitch of the ballscrew.
  - \f$K_b\f$ is the motor back emf constant.
  - L is the motor armature inductance.
  - R is the motor armature resistance.
  - \f$K_t\f$ is the motor torque constant.
  - \f$ M_c\f$, \f$N_c\f$ and \f$G_c\f$ are from Find_Mc_Nc_Gc().
*/
ReturnMatrix Stewart::ForwardDyn_AD(const ColumnVector Command, const Real t)
{
    Matrix _ddq;
    Matrix Nc,Gc,Mc, tmp1,tmp2;
    
    Find_Mc_Nc_Gc(Mc,Nc,Gc);
    
    tmp1 = (Command - (Jacobian.i()*dq*(2*M_PI/p)*Kb));
    tmp2 = (IdentityMatrix(6)*Kt/L*exp(-R*t/L))*tmp1;

    _ddq = Mc.i()*(tmp2 - Nc - Gc);
    
    _ddq.Release();  
    return _ddq;
}

#ifdef use_namespace
}
#endif

