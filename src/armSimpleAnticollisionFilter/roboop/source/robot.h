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
   -Removed class mlink. DH and modified DH parameters are now included in link.
   -Created virtual class Robot_basic which is now the base class of Robot and 
    mRobot.
   -Removed classes RobotMotor and mRobotMotor. Motors effect are now included
    in classes Robot and mRobot. Code using the old motor class whould need to 
    be change by changing RobotMotor by Robot and mRobotMotor by mRobot.
   -Added classes mRobot_min_para (modified DH parameters) and IO_matrix_file.
   -Created a new torque member function that allowed to have load on last link
    (Robot_basic, Robot, mRobot, mRobot_min_para).
   -Added the following member functions in class Robot_basic:
     void kine_pd(Matrix & Rot, ColumnVector & pos, ColumnVector & posdot)const;
     void kine_pd(Matrix & Rot, ColumnVector & pos, ColumnVector & posdot, int j)const;
     ReturnMatrix kine_pd(void)const;
     ReturnMatrix kine_pd(int j)const;
    These functions are like the kine(), but with speed calculation.
   -Added labels theta_min and theta_max in class Link.

2003/04/29: Etienne Lachance
   -All gnugrah.cpp definitions are now in gnugraph.h.
   -The following ColumnVector are now part of class Robot_basic: *dw, *dwp, *dvp, *da, 
    *df, *dn, *dF, *dN, *dp.
   -Added functons Robot_basic::jacobian_DLS_inv.
   -Added z0 in Robot_basic.
   -Fix kine_pd function.

2003/08/22: Etienne Lachance
   -Added parameter converge in member function inv_kin.

2004/01/23: Etienne Lachance
   -Added member function G() (gravity torque), and H() (corriolis torque)
    on all robot class.
   -Commun definitions are now in include file utils.h.
   -Added const in non reference argument for all functions prototypes.

2004/03/01: Etienne Lachance
   -Added non member function perturb_robot.

2004/03/21: Etienne Lachance
   -Added the following member functions: get_theta_min, get_theta_max,
    get_mc, get_r, get_p, get_m, get_Im, get_Gr, get_B, get_Cf, get_I,
    set_m, set_mc, set_r, set_Im, set_B, set_Cf, set_I.

2004/04/26: Etienne Lachance and Vincent Drolet
   -Added member functions inv_kin_rhino and inv_kin_puma.

2004/05/21: Etienne Lachance
   -Added Doxygen comments.

2004/07/01: Ethan Tira-Thompson
    -Added support for newmat's use_namespace #define, using ROBOOP namespace

2004/07/02: Etienne Lachance
    -Added joint_offset variable in class Link and Link member functions 
     get_joint_offset. Idea proposed by Ethan Tira-Thompson.

2004/07/16: Ethan Tira-Thompson
    -Added Link::immobile flag and accessor functions
    -Added get_available_q* functions
    -inv_kin functions are now all virtual
    -Added parameters to jacobian and inv_kin functions to work with frames
     other than the end effector

2004/10/02: Etienne Lachance
    -Added Schlling for analytic inverse kinematic.

2005/11/06: Etienne Lachance
    - No need to provide a copy constructor and the assignment operator 
      (operator=) for Link class. Instead we use the one provide by the
      compiler.
    -No need to provide an assignment operator for Robot, mRobot and
     mRobot_min_para classes.
-------------------------------------------------------------------------------
*/

#ifndef __cplusplus
#error Must use C++ for the type Robot
#endif
#ifndef ROBOT_H
#define ROBOT_H

/*!
  @file robot.h
  @brief Robots class definitions.
*/

//! @brief RCS/CVS version.
static const char header_rcsid[] = "$Id: robot.h,v 1.1 2007/07/24 16:03:10 amaldo Exp $";

#include "utils.h"

#ifdef use_namespace
namespace ROBOOP {
  using namespace NEWMAT;
#endif


/*!
  @class Link
  @brief Link definitions.

  A n degree of freedom (dof) serial manipulator is composed of n links. This class
  describe the property of a link. A n dof robot has n instance of the class Link.
*/
class Link  
{
    friend class Robot_basic;
    friend class Robot;
    friend class mRobot;
    friend class mRobot_min_para;

public:
   Link(const int jt = 0, const Real it = 0.0, const Real id = 0.0,
        const Real ia = 0.0, const Real ial = 0.0, const Real theta_min = -M_PI/2,
        const Real theta_max = M_PI/2, const Real it_off = 0.0, const Real mass = 1.0,
        const Real cmx = 0.0, const Real cmy = 0.0, const Real cmz = 0.0,
        const Real ixx = 0.0, const Real ixy = 0.0, const Real ixz = 0.0,
        const Real iyy = 0.0, const Real iyz = 0.0, const Real izz = 0.0,
        const Real iIm = 0.0, const Real iGr = 0.0, const Real iB = 0.0,
        const Real iCf = 0.0, const bool dh = true, const bool min_inertial_para = false,
	const bool immobile = false);
   ~Link(){}                                            //!< Destructor.
   void transform(const Real q);
   bool get_DH(void) const {return DH; }                //!< Return DH value.
   int get_joint_type(void) const { return joint_type; }//!< Return the joint type.
   Real get_theta(void) const { return theta; }         //!< Return theta.
   Real get_d(void) const { return d; }                 //!< Return d.
   Real get_a(void) const { return a; }                 //!< Return a.
   Real get_alpha(void) const { return alpha; }         //!< Return alpha.
   Real get_q(void) const;
   Real get_theta_min(void) const { return theta_min; } //!< Return theta_min.
   Real get_theta_max(void) const { return theta_max; } //!< Return theta_max.
   Real get_joint_offset(void) const { return joint_offset; } //!< Return joint_offset.
   ReturnMatrix get_mc(void) { return mc; }             //!< Return mc.
   ReturnMatrix get_r(void) { return r; }               //!< Return r.
   ReturnMatrix get_p(void) const { return p; }         //!< Return p.
   Real get_m(void) const { return m; }                 //!< Return m.
   Real get_Im(void) const { return Im; }               //!< Return Im.
   Real get_Gr(void) const { return Gr; }               //!< Return Gr.
   Real get_B(void) const { return B; }                 //!< Return B.
   Real get_Cf(void) const { return Cf; }               //!< Return Cf.
   ReturnMatrix get_I(void) const { return I; }         //!< Return I.
   bool get_immobile(void) const { return immobile; }   //!< Return immobile.
   void set_m(const Real m_) { m = m_; }                //!< Set m.
   void set_mc(const ColumnVector & mc_);               //!< Set mc.
   void set_r(const ColumnVector & r_);                 //!< Set r.
   void set_Im(const Real Im_) { Im = Im_; }            //!< Set Im.
   void set_B(const Real B_) { B = B_; }                //!< Set B.
   void set_Cf(const Real Cf_) { Cf = Cf_; }            //!< Set Cf.
   void set_I(const Matrix & I);                        //!< Set I.
   void set_immobile(bool im) { immobile=im; }          //!< Set immobile.

   Matrix R;          //!< Orientation matrix of actual link w.r.t to previous link.
   Real qp,           //!< Joint velocity.
        qpp;          //!< Joint acceleration.

private:
   int joint_type;    //!< Joint type.
   Real theta,        //!< theta DH parameter.
        d,            //!< d DH parameter.
        a,            //!< a DH parameter.
        alpha,        //!< alpha DH parameter.
        theta_min,    //!< Min joint angle.
        theta_max,    //!< Max joint angle. 
        joint_offset; //!< Offset in joint angle (rotoide and prismatic).
   bool DH,           //!< DH notation(true) or DH modified notation.
        min_para;     //!< Minimum inertial parameter.
   ColumnVector r,    //!< Position of center of mass w.r.t. link coordinate system (min_para=F).
                p;    //!< Position vector of actual link w.r.t to previous link.
   Real m,            //!< Mass of the link.
   Im,                //!< Motor Inertia.
   Gr,                //!< Gear Ratio.
   B,                 //!< Viscous coefficient.
   Cf;                //!< Coulomb fiction coefficient.
   ColumnVector mc;   //!< Mass \f$\times\f$ center of gravity (used if min_para = true).
   Matrix I;          //!< Inertia matrix w.r.t. center of mass and link coordinate system orientation.
   bool immobile;     //!< true if the joint is to be considered locked - ignored for inverse kinematics, but can still be reassigned through transform
};

/*!
  @class Robot_basic
  @brief Virtual base robot class.
*/
class Robot_basic {
   friend class Robot;
   friend class mRobot;
   friend class mRobot_min_para;
   friend class Robotgl;
   friend class mRobotgl;
public:
   Robot_basic(const int ndof = 1, const bool dh_parameter = false,
               const bool min_inertial_para = false);
   Robot_basic(const Matrix & initrobot_motor, const bool dh_parameter = false,
               const bool min_inertial_para = false);
   Robot_basic(const Matrix & initrobot, const Matrix & initmotor,
               const bool dh_parameter = false, const bool min_inertial_para = false);
   Robot_basic(const std::string & filename, const std::string & robotName,
               const bool dh_parameter = false, const bool min_inertial_para = false);
   Robot_basic(const Robot_basic & x);
   virtual ~Robot_basic();
   Robot_basic & operator=(const Robot_basic & x);

   Real get_q(const int i) const {            //!< Return joint i position.
      if(i < 1 || i > dof) error("i must be 1 <= i <= dof");
      return links[i].get_q();
   }
   bool get_DH()const { return links[1].DH; } //!< Return true if in DH notation, false otherwise.
   int get_dof()const { return dof; }         //!< Return dof.
   int get_available_dof()const { return get_available_dof(dof); } //!<Counts number of currently non-immobile links
   int get_available_dof(const int endlink)const;
   int get_fix()const { return fix; }         //!< Return fix.
   ReturnMatrix get_q(void)const;
   ReturnMatrix get_qp(void)const;
   ReturnMatrix get_qpp(void)const;
   ReturnMatrix get_available_q(void)const { return get_available_q(dof); } //!< Return the joint position vector of available (non-immobile) joints.
   ReturnMatrix get_available_qp(void)const { return get_available_qp(dof); } //!< Return the joint velocity vector of available (non-immobile) joints.
   ReturnMatrix get_available_qpp(void)const { return get_available_qpp(dof); } //!< Return the joint acceleration vector of available (non-immobile) joints.
   ReturnMatrix get_available_q(const int endlink)const;
   ReturnMatrix get_available_qp(const int endlink)const;
   ReturnMatrix get_available_qpp(const int endlink)const;
   void set_q(const ColumnVector & q);
   void set_q(const Matrix & q);
   void set_q(const Real q, const int i) {     //!< Set joint i position.
      if(i < 1 || i > dof) error("i must be 1 <= i <= dof");
      links[i].transform(q);
   }
   void set_qp(const ColumnVector & qp);
   void set_qpp(const ColumnVector & qpp);
   void kine(Matrix & Rot, ColumnVector & pos)const;
   void kine(Matrix & Rot, ColumnVector & pos, const int j)const;
   ReturnMatrix kine(void)const;
   ReturnMatrix kine(const int j)const;
   ReturnMatrix kine_pd(const int ref=0)const;
   virtual void kine_pd(Matrix & Rot, ColumnVector & pos,
                        ColumnVector & pos_dot, const int ref)const=0; 
   virtual void robotType_inv_kin() = 0;
   virtual ReturnMatrix inv_kin(const Matrix & Tobj, const int mj = 0);
//!< Numerical inverse kinematics.  See inv_kin(const Matrix & Tobj, const int mj, const int endlink, bool & converge)
   ReturnMatrix inv_kin(const Matrix & Tobj, const int mj, 
			bool & converge) {return inv_kin(Tobj,mj,dof,converge);} 
   virtual ReturnMatrix inv_kin(const Matrix & Tobj, const int mj, const int endlink, bool & converge);
   virtual ReturnMatrix inv_kin_rhino(const Matrix & Tobj, bool & converge) = 0;
   virtual ReturnMatrix inv_kin_puma(const Matrix & Tobj, bool & converge) = 0;
   virtual ReturnMatrix inv_kin_schilling(const Matrix & Tobj, bool & converge) = 0;
   virtual ReturnMatrix jacobian(const int ref=0)const { return jacobian(dof,ref); } //!< Jacobian of mobile links expressed at frame ref.
   virtual ReturnMatrix jacobian(const int endlink, const int ref)const = 0;
   virtual ReturnMatrix jacobian_dot(const int ref=0)const = 0;
   ReturnMatrix jacobian_DLS_inv(const double eps, const double lambda_max, const int ref=0)const;
   virtual void dTdqi(Matrix & dRot, ColumnVector & dp, const int i) = 0;
   virtual ReturnMatrix dTdqi(const int i) = 0;
   ReturnMatrix acceleration(const ColumnVector & q, const ColumnVector & qp,
                             const ColumnVector & tau);
   ReturnMatrix acceleration(const ColumnVector & q, const ColumnVector & qp,
                             const ColumnVector & tau, const ColumnVector & Fext,
                             const ColumnVector & Next);
   ReturnMatrix inertia(const ColumnVector & q);
   virtual ReturnMatrix torque_novelocity(const ColumnVector & qpp) = 0;
   virtual ReturnMatrix torque(const ColumnVector & q, const ColumnVector & qp,
                               const ColumnVector & qpp) = 0;
   virtual ReturnMatrix torque(const ColumnVector & q, const ColumnVector & qp,
                               const ColumnVector & qpp,
                               const ColumnVector & Fext_,
                               const ColumnVector & Next_) = 0;
   virtual void delta_torque(const ColumnVector & q, const ColumnVector & qp,
                             const ColumnVector & qpp, const ColumnVector & dq,
                             const ColumnVector & dqp, const ColumnVector & dqpp,
                             ColumnVector & torque, ColumnVector & dtorque) =0;
   virtual void dq_torque(const ColumnVector & q, const ColumnVector & qp,
                          const ColumnVector & qpp, const ColumnVector & dq,
                          ColumnVector & torque, ColumnVector & dtorque) =0;
   virtual void dqp_torque(const ColumnVector & q, const ColumnVector & qp,
                           const ColumnVector & dqp, ColumnVector & torque,
                           ColumnVector & dtorque) =0;
   ReturnMatrix dtau_dq(const ColumnVector & q, const ColumnVector & qp,
                        const ColumnVector & qpp);
   ReturnMatrix dtau_dqp(const ColumnVector & q, const ColumnVector & qp);
   virtual ReturnMatrix G() = 0;
   virtual ReturnMatrix C(const ColumnVector & qp) = 0;
   void error(const std::string & msg1) const;

   ColumnVector *w, *wp, *vp, *a, *f, *f_nv, *n, *n_nv, *F, *N, *p, *pp,
   *dw, *dwp, *dvp, *da, *df, *dn, *dF, *dN, *dp, 
       z0,      //!< Axis vector at each joint.
       gravity; //!< Gravity vector.
   Matrix *R;   //!< Temprary rotation matrix.
   Link *links; //!< Pointer on Link cclass.

private:
   void cleanUpPointers();

   //! enum EnumRobotType
   enum EnumRobotType 
   { 
       DEFAULT = 0,   //!< Default robot familly.
       RHINO = 1,     //!< Rhino familly.
       PUMA = 2,      //!< Puma familly.
       SCHILLING = 3  //!< Schilling familly
   };
   EnumRobotType robotType; //!< Robot type.
   int dof,                 //!< Degree of freedom.
       fix;                 //!< Virtual link, used with modified DH notation.
};

/*!
  @class Robot
  @brief DH notation robot class.
*/
class Robot : public Robot_basic
{
public:
   Robot(const int ndof=1);
   Robot(const Matrix & initrobot);
   Robot(const Matrix & initrobot, const Matrix & initmotor);
   Robot(const Robot & x);
   Robot(const std::string & filename, const std::string & robotName);
   virtual ~Robot(){}   //!< Destructor.
   virtual void robotType_inv_kin();
   virtual void kine_pd(Matrix & Rot, ColumnVector & pos,
                        ColumnVector & pos_dot, const int ref)const;
   ReturnMatrix inv_kin(const Matrix & Tobj, const int mj = 0);
   virtual ReturnMatrix inv_kin(const Matrix & Tobj, const int mj, const int endlink, bool & converge);
   virtual ReturnMatrix inv_kin_rhino(const Matrix & Tobj, bool & converge);
   virtual ReturnMatrix inv_kin_puma(const Matrix & Tobj, bool & converge);
   virtual ReturnMatrix inv_kin_schilling(const Matrix & Tobj, bool & converge);
   virtual ReturnMatrix jacobian(const int ref=0)const { return jacobian(dof,ref); } //!< Jacobian of mobile links expressed at frame ref.
   virtual ReturnMatrix jacobian(const int endlink, const int ref)const;
   virtual ReturnMatrix jacobian_dot(const int ref=0)const;
   virtual void dTdqi(Matrix & dRot, ColumnVector & dp, const int i);
   virtual ReturnMatrix dTdqi(const int i);
   virtual ReturnMatrix torque(const ColumnVector & q, const ColumnVector & qp,
                               const ColumnVector & qpp);
   virtual ReturnMatrix torque(const ColumnVector & q, const ColumnVector & qp,
                               const ColumnVector & qpp,
                               const ColumnVector & Fext_,
                               const ColumnVector & Next_);
   virtual ReturnMatrix torque_novelocity(const ColumnVector & qpp);
   virtual void delta_torque(const ColumnVector & q, const ColumnVector & qp,
                             const ColumnVector & qpp, const ColumnVector & dq,
                             const ColumnVector & dqp, const ColumnVector & dqpp,
                             ColumnVector & ltorque, ColumnVector & dtorque);
   virtual void dq_torque(const ColumnVector & q, const ColumnVector & qp,
                          const ColumnVector & qpp, const ColumnVector & dq,
                          ColumnVector & torque, ColumnVector & dtorque);
   virtual void dqp_torque(const ColumnVector & q, const ColumnVector & qp,
                           const ColumnVector & dqp, ColumnVector & torque,
                           ColumnVector & dtorque);
   virtual ReturnMatrix G();
   virtual ReturnMatrix C(const ColumnVector & qp);
};

// ---------- R O B O T   M O D I F I E D   DH   N O T A T I O N --------------

/*!
  @class mRobot
  @brief Modified DH notation robot class.
*/
class mRobot : public Robot_basic {
public:
   mRobot(const int ndof=1);
   mRobot(const Matrix & initrobot_motor);
   mRobot(const Matrix & initrobot, const Matrix & initmotor);
   mRobot(const mRobot & x);
   mRobot(const std::string & filename, const std::string & robotName);
   virtual ~mRobot(){}    //!< Destructor.
   virtual void robotType_inv_kin();
   ReturnMatrix inv_kin(const Matrix & Tobj, const int mj = 0);
   virtual ReturnMatrix inv_kin(const Matrix & Tobj, const int mj, const int endlink, bool & converge);
   virtual ReturnMatrix inv_kin_rhino(const Matrix & Tobj, bool & converge);
   virtual ReturnMatrix inv_kin_puma(const Matrix & Tobj, bool & converge);
   virtual ReturnMatrix inv_kin_schilling(const Matrix & Tobj, bool & converge);
   virtual void kine_pd(Matrix & Rot, ColumnVector & pos,
                        ColumnVector & pos_dot, const int ref)const;
   virtual ReturnMatrix jacobian(const int ref=0)const { return jacobian(dof,ref); } //!< Jacobian of mobile links expressed at frame ref.
   virtual ReturnMatrix jacobian(const int endlink, const int ref)const;
   virtual ReturnMatrix jacobian_dot(const int ref=0)const;
   virtual void dTdqi(Matrix & dRot, ColumnVector & dp, const int i);
   virtual ReturnMatrix dTdqi(const int i);
   virtual ReturnMatrix torque(const ColumnVector & q, const ColumnVector & qp,
                               const ColumnVector & qpp);
   virtual ReturnMatrix torque(const ColumnVector & q, const ColumnVector & qp,
                               const ColumnVector & qpp,
                               const ColumnVector & Fext_,
                               const ColumnVector & Next_);
   virtual ReturnMatrix torque_novelocity(const ColumnVector & qpp);
   virtual void delta_torque(const ColumnVector & q, const ColumnVector & qp,
                             const ColumnVector & qpp, const ColumnVector & dq,
                             const ColumnVector & dqp, const ColumnVector & dqpp,
                             ColumnVector & torque, ColumnVector & dtorque);
   virtual void dq_torque(const ColumnVector & q, const ColumnVector & qp,
                          const ColumnVector & qpp, const ColumnVector & dq,
                          ColumnVector & torque, ColumnVector & dtorque);
   virtual void dqp_torque(const ColumnVector & q, const ColumnVector & qp,
                           const ColumnVector & dqp, ColumnVector & torque,
                           ColumnVector & dtorque);
   virtual ReturnMatrix G();
   virtual ReturnMatrix C(const ColumnVector & qp);
};

// --- R O B O T  DH  M O D I F I E D,  M I N I M U M   P A R A M E T E R S ---

/*!
  @class mRobot_min_para
  @brief Modified DH notation and minimal inertial parameters robot class.
*/
class mRobot_min_para : public Robot_basic 
{
public:
   mRobot_min_para(const int ndof=1);
   mRobot_min_para(const Matrix & dhinit);
   mRobot_min_para(const Matrix & initrobot, const Matrix & initmotor);
   mRobot_min_para(const mRobot_min_para & x);
   mRobot_min_para(const std::string & filename, const std::string & robotName);
   virtual ~mRobot_min_para(){}    //!< Destructor.
   virtual void robotType_inv_kin();
   ReturnMatrix inv_kin(const Matrix & Tobj, const int mj = 0);
   virtual ReturnMatrix inv_kin(const Matrix & Tobj, const int mj, const int endlink, bool & converge);
   virtual ReturnMatrix inv_kin_rhino(const Matrix & Tobj, bool & converge);
   virtual ReturnMatrix inv_kin_puma(const Matrix & Tobj, bool & converge);
   virtual ReturnMatrix inv_kin_schilling(const Matrix & Tobj, bool & converge);
   virtual void kine_pd(Matrix & Rot, ColumnVector & pos,
                        ColumnVector & pos_dot, const int ref=0)const;
   virtual ReturnMatrix jacobian(const int ref=0)const { return jacobian(dof,ref); } //!< Jacobian of mobile links expressed at frame ref.
   virtual ReturnMatrix jacobian(const int endlink, const int ref)const;
   virtual ReturnMatrix jacobian_dot(const int ref=0)const;
   virtual void dTdqi(Matrix & dRot, ColumnVector & dp, const int i);
   virtual ReturnMatrix dTdqi(const int i);
   virtual ReturnMatrix torque(const ColumnVector & q, const ColumnVector & qp,
                               const ColumnVector & qpp);
   virtual ReturnMatrix torque(const ColumnVector & q,
                               const ColumnVector & qp,
                               const ColumnVector & qpp,
                               const ColumnVector & Fext_,
                               const ColumnVector & Next_);
   virtual ReturnMatrix torque_novelocity(const ColumnVector & qpp);
   virtual void delta_torque(const ColumnVector & q, const ColumnVector & qp,
                             const ColumnVector & qpp, const ColumnVector & dq,
                             const ColumnVector & dqp, const ColumnVector & dqpp,
                             ColumnVector & torque, ColumnVector & dtorque);
   virtual void dq_torque(const ColumnVector & q, const ColumnVector & qp,
                          const ColumnVector & qpp, const ColumnVector & dq,
                          ColumnVector & torque, ColumnVector & dtorque);
   virtual void dqp_torque(const ColumnVector & q, const ColumnVector & qp,
                           const ColumnVector & dqp, ColumnVector & torque,
                           ColumnVector & dtorque);
   virtual ReturnMatrix G();
   virtual ReturnMatrix C(const ColumnVector & qp);
};

void perturb_robot(Robot_basic & robot, const double f = 0.1);

bool Rhino_DH(const Robot_basic & robot);
bool Puma_DH(const Robot_basic & robot);
bool Schilling_DH(const Robot_basic & robot);

bool Rhino_mDH(const Robot_basic  & robot);
bool Puma_mDH(const Robot_basic  & robot);
bool Schilling_mDH(const Robot_basic & robot);

#ifdef use_namespace
}
#endif

#endif 

