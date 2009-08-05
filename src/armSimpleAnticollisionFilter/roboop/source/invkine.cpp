/*
ROBOOP -- A robotics object oriented package in C++
Copyright (C) 2004  Etienne Lachance

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

2004/04/19: Vincent Drolet
   -Added Robot::inv_kin_rhino and Robot::inv_kin_puma member functions.

2004/04/20: Etienne Lachance
   -Added try, throw, catch statement in Robot::inv_kin_rhino and 
    Robot::inv_kin_puma in order to avoid singularity.

2004/05/21: Etienne Lachance
   -Added Doxygen documentation.

2004/07/01: Ethan Tira-Thompson
    -Added support for newmat's use_namespace #define, using ROBOOP namespace
    -Fixed warnings regarding atan2 when using float as Real type

2004/07/16: Ethan Tira-Thompson
    -If USING_FLOAT is set from newmat's include.h, ITOL is 1e-4 instead of 1e-6
     Motivation was occasional failures to converge when requiring 1e-6
     precision from floats using prismatic joints with ranges to 100's
    -A few modifications to support only solving for mobile joints in chain
    -Can now do inverse kinematics for frames other than end effector

2004/12/23: Brian Galardo, Jean-Pascal Joary, Etienne Lachance
   -Added Robot::inv_schilling, mRobot::inv_schilling and mRobot_min_para::inv_schilling
    member functions.
   -Fixed previous bug on Rhino and Puma inverse kinematics.
-------------------------------------------------------------------------------
*/

/*!
  @file invkine.cpp
  @brief Inverse kinematics solutions.
*/

//! @brief RCS/CVS version.
static const char rcsid[] = "$Id: invkine.cpp,v 1.1 2007/07/24 16:03:09 amaldo Exp $";

#include <stdexcept>
#include "robot.h"

#ifdef use_namespace
namespace ROBOOP {
  using namespace NEWMAT;
#endif

#define NITMAX 1000  //!< def maximum number of iterations in inv_kin 
#ifdef USING_FLOAT //from newmat's include.h
#  define ITOL   1e-4  //!< def tolerance for the end of iterations in inv_kin 
#else
#  define ITOL   1e-6  //!< def tolerance for the end of iterations in inv_kin 
#endif

ReturnMatrix Robot_basic::inv_kin(const Matrix & Tobj, const int mj)
//!  @brief Overload inv_kin function.
{
   bool converge = false;
   return inv_kin(Tobj, mj, dof, converge);
}


ReturnMatrix Robot_basic::inv_kin(const Matrix & Tobj, const int mj, const int endlink, bool & converge)
/*!
  @brief Numerical inverse kinematics.

  @param Tobj: Transformation matrix expressing the desired end effector pose.
  @param mj: Select algorithm type, 0: based on Jacobian, 1: based on derivative of T.
  @param converge: Indicate if the algorithm converge.
  @param endlink: the link to pretend is the end effector 

  The joint position vector before the inverse kinematics is returned if the 
  algorithm does not converge.
*/
{
   ColumnVector qPrev, qout, dq, q_tmp;
   Matrix B, M;
   UpperTriangularMatrix U;

   qPrev = get_available_q();
   qout = qPrev;
   q_tmp = qout;

   converge = false;
   if(mj == 0) {  // Jacobian based
      Matrix Ipd, A, B(6,1);
      for(int j = 1; j <= NITMAX; j++) {
         Ipd = (kine(endlink)).i()*Tobj;
         B(1,1) = Ipd(1,4);
         B(2,1) = Ipd(2,4);
         B(3,1) = Ipd(3,4);
         B(4,1) = Ipd(3,2);
         B(5,1) = Ipd(1,3);
         B(6,1) = Ipd(2,1);
         A = jacobian(endlink,endlink);
         QRZ(A,U);
         QRZ(A,B,M);
         dq = U.i()*M;

	 while(dq.MaximumAbsoluteValue() > 1)
	     dq /= 10;

         for(int k = 1; k<= dq.nrows(); k++)
            qout(k)+=dq(k);
         set_q(qout);

         if (dq.MaximumAbsoluteValue() < ITOL)
         {
            converge = true;
            break;
         }
      }
   } else {  // using partial derivative of T
      int adof=get_available_dof(endlink);
      Matrix A(12,adof);
      for(int j = 1; j <= NITMAX; j++) {
         B = (Tobj-kine(endlink)).SubMatrix(1,3,1,4).AsColumn();
         int k=1;
         for(int i = 1; i<=dof && k<=adof; i++) {
            if(links[i].immobile)
               continue;
            A.SubMatrix(1,12,k,k) = dTdqi(i).SubMatrix(1,3,1,4).AsColumn();
            k++;
         }
         QRZ(A,U);
         QRZ(A,B,M);
         dq = U.i()*M;

	 while(dq.MaximumAbsoluteValue() > 1)
	     dq /= 10;

         for(k = 1; k<=adof; k++)
            qout(k)+=dq(k);
         set_q(qout);
         if (dq.MaximumAbsoluteValue() < ITOL)
         {
            converge = true;
            break;
         }
      }
   }

   if(converge)
   {
      // Make sure that: -pi < qout <= pi for revolute joints
      for(int i = 1; i <= dof; i++)
      {
         if(links[i].immobile)
            continue;
         if(links[i].get_joint_type() == 0) {
	     qout(i) = fmod(qout(i), 2*M_PI);
         }
      }
      set_q(qPrev);
      qout.Release();
      return qout;
   }
   else
   {
      set_q(qPrev);
      q_tmp.Release();
      return q_tmp;
   }
}

// ---------------------  R O B O T   DH   N O T A T I O N  --------------------------

ReturnMatrix Robot::inv_kin(const Matrix & Tobj, const int mj)
//!  @brief Overload inv_kin function.
{
   bool converge = false;
   return inv_kin(Tobj, mj, dof, converge);
}


ReturnMatrix Robot::inv_kin(const Matrix & Tobj, const int mj, const int endlink, bool & converge)
/*!
  @brief Inverse kinematics solutions.

  The solution is based on the analytic inverse kinematics if robot type (familly)
  is Rhino or Puma, otherwise used the numerical algoritm defined in Robot_basic
  class.
*/
{
    switch (robotType) {
	case RHINO:
	    return inv_kin_rhino(Tobj, converge);
	    break;
	case PUMA:
	    return inv_kin_puma(Tobj, converge);
	    break;
	case SCHILLING:
	    return inv_kin_schilling(Tobj, converge);
	    break;
	default:
	    return Robot_basic::inv_kin(Tobj, mj, endlink, converge);
    }
}


ReturnMatrix Robot::inv_kin_rhino(const Matrix & Tobj, bool & converge)
/*!
  @brief Analytic Rhino inverse kinematics.

  converge will be false if the desired end effector pose is outside robot range.
*/
{
    ColumnVector qout(5), q_actual;
    q_actual = get_q();

    try
    {
        Real theta[6] , diff1, diff2, tmp,
        angle , L=0.0 , M=0.0 , K=0.0 , H=0.0 , G=0.0 ;

        // Calcul les deux angles possibles
        theta[0] = atan2(Tobj(2,4),
                         Tobj(1,4));

        theta[1] = atan2(-Tobj(2,4),
                         -Tobj(1,4))  ;

        diff1 = fabs(q_actual(1)-theta[0]) ;		
        if (diff1 > M_PI)
            diff1 = 2*M_PI - diff1;

        diff2 = fabs(q_actual(1)-theta[1]);
        if (diff2 > M_PI)
            diff2 = 2*M_PI - diff2 ;

        // Prend l'angle le plus proche de sa position actuel
        if (diff1 < diff2)		   
            theta[1] = theta[0] ;

        theta[5] = atan2(sin(theta[1])*Tobj(1,1) - cos(theta[1])*Tobj(2,1), 
                         sin(theta[1])*Tobj(1,2) - cos(theta[1])*Tobj(2,2));

        // angle = theta1 +theta2 + theta3
        angle = atan2(-1*cos(theta[1])*Tobj(1,3) - sin(theta[1])*Tobj(2,3),
                      -1*Tobj(3,3));

        L = cos(theta[1])*Tobj(1,4) + 
            sin(theta[1])*Tobj(2,4) + 
            links[5].d*sin(angle) - 
            links[4].a*cos(angle);
        M = links[1].d - 
            Tobj(3,4) - 
            links[5].d*cos(angle) - 
            links[4].a*sin(angle);
        K = (L*L + M*M - links[3].a*links[3].a - 
             links[2].a*links[2].a) / 
            (2 * links[3].a * links[2].a);

        tmp = 1-K*K;
        if (tmp < 0)
            throw out_of_range("sqrt of negative number not allowed.");

        theta[0] = atan2( sqrt(tmp) , K );
        theta[3] = atan2( -sqrt(tmp) , K );	

        diff1 = fabs(q_actual(3)-theta[0]) ;
        if (diff1 > M_PI)
            diff1 = 2*M_PI - diff1 ;

        diff2 = fabs(q_actual(3)-theta[3]);
        if (diff2 > M_PI)
            diff2 = 2*M_PI - diff2 ;

        if (diff1 < diff2)
            theta[3] = theta[0] ;

        H = cos(theta[3]) * links[3].a + links[2].a;
        G = sin(theta[3]) * links[3].a;

        theta[2] = atan2( M , L ) - atan2( G , H );
        theta[4] = atan2( -1*cos(theta[1])*Tobj(1,3) - sin(theta[1])*Tobj(2,3) , 
                          -1*Tobj(3,3)) - theta[2] - theta[3] ;

        qout(1) = theta[1];
        qout(2) = theta[2];
        qout(3) = theta[3];
        qout(4) = theta[4];
        qout(5) = theta[5];

        converge = true;
    }
    catch(std::out_of_range & e)
    {
        converge = false; 
        qout = q_actual;
    }

    qout.Release();
    return qout;
}


ReturnMatrix Robot::inv_kin_puma(const Matrix & Tobj, bool & converge)
/*!
  @brief Analytic Puma inverse kinematics.

  converge will be false if the desired end effector pose is outside robot range.
*/
{
    ColumnVector qout(6), q_actual;
    q_actual = get_q();

    try
    {  
        Real theta[7] , diff1, diff2, tmp,
        A = 0.0 , B = 0.0 , C = 0.0 , D =0.0, Ro = 0.0,
        H = 0.0 , L = 0.0 , M = 0.0;

        // Removed d6 component because in the Puma inverse kinematics solution 
        // d6 = 0. 
        if (links[6].d)
        {
            ColumnVector tmpd6(3); Matrix tmp;
            tmpd6(1)=0; tmpd6(2)=0; tmpd6(3)=links[6].d;
            tmpd6 = Tobj.SubMatrix(1,3,1,3)*tmpd6;
            Tobj.SubMatrix(1,3,4,4) = Tobj.SubMatrix(1,3,4,4) - tmpd6;
        }

        tmp = Tobj(2,4)*Tobj(2,4) + Tobj(1,4)*Tobj(1,4);
        if (tmp < 0)
            throw std::out_of_range("sqrt of negative number not allowed.");

        Ro = sqrt(tmp);
        D = (links[2].d+links[3].d) / Ro;

        tmp = 1-D*D;
        if (tmp < 0)
            throw std::out_of_range("sqrt of negative number not allowed.");

        //Calcul les deux angles possibles
        theta[0] =  atan2(Tobj(2,4),Tobj(1,4)) - atan2(D, sqrt(tmp));	 
        //Calcul les deux angles possibles
        theta[1] =  atan2(Tobj(2,4),Tobj(1,4)) - atan2(D , -sqrt(tmp));

        diff1 = fabs(q_actual(1)-theta[0]);
        if (diff1 > M_PI)
            diff1 = 2*M_PI - diff1;

        diff2 = fabs(q_actual(1)-theta[1]);
        if (diff2 > M_PI)
            diff2 = 2*M_PI - diff2;

        // Prend l'angle le plus proche de sa position actuel
        if (diff1 < diff2)
            theta[1] = theta[0];

        tmp = links[3].a*links[3].a + links[4].d*links[4].d;
        if (tmp < 0)
            throw std::out_of_range("sqrt of negative number not allowed.");

        Ro = sqrt(tmp);
        B = atan2(links[4].d,links[3].a);
        C = Tobj(1,4)*Tobj(1,4) + 
            Tobj(2,4)*Tobj(2,4) + 
            (Tobj(3,4)-links[1].d)*(Tobj(3,4)-links[1].d) - 
            (links[2].d + links[3].d)*(links[2].d + links[3].d) - 
            links[2].a*links[2].a - 
            links[3].a*links[3].a - 
            links[4].d*links[4].d; 
        A = C / (2*links[2].a);

        tmp = 1-A/Ro*A/Ro;
        if (tmp < 0)
            throw std::out_of_range("sqrt of negative number not allowed.");

        theta[0] = atan2(sqrt(tmp) , A/Ro) + B;
        theta[3] = atan2(-sqrt(tmp) , A/Ro) + B;

        diff1 = fabs(q_actual(3)-theta[0]);
        if (diff1 > M_PI)
            diff1 = 2*M_PI - diff1 ;

        diff2 = fabs(q_actual(3)-theta[3]);
        if (diff2 > M_PI)
            diff1 = 2*M_PI - diff2;

        //Prend l'angle le plus proche de sa position actuel
        if (diff1 < diff2)
            theta[3] = theta[0];

        H = cos(theta[1])*Tobj(1,4) + sin(theta[1])*Tobj(2,4);
        L = sin(theta[3])*links[4].d + cos(theta[3])*links[3].a + links[2].a;
        M = cos(theta[3])*links[4].d - sin(theta[3])*links[3].a;

        theta[2] = atan2( M , L ) - atan2(Tobj(3,4)-links[1].d , H );

        theta[0] = atan2( -sin(theta[1])*Tobj(1,3) + cos(theta[1])*Tobj(2,3) , 
                          cos(theta[2] + theta[3]) * 
                          (cos(theta[1]) * Tobj(1,3) + sin(theta[1])*Tobj(2,3)) 
                          - (sin(theta[2]+theta[3])*Tobj(3,3)) );

        theta[4] = atan2(-1*(-sin(theta[1])*Tobj(1,3) + cos(theta[1])*Tobj(2,3)), 
                         -cos(theta[2] + theta[3]) * 
                         (cos(theta[1]) * Tobj(1,3) + sin(theta[1])*Tobj(2,3)) 
                         + (sin(theta[2]+theta[3])*Tobj(3,3)) );

        diff1 = fabs(q_actual(4)-theta[0]);
        if (diff1 > M_PI)
            diff1 = 2*M_PI - diff1;

        diff2 = fabs(q_actual(4)-theta[4]);
        if (diff2 > M_PI)
            diff2 = 2*M_PI - diff2;

        // Prend l'angle le plus proche de sa position actuel
        if (diff1 < diff2)
            theta[4] = theta[0];

        theta[5] = atan2( cos(theta[4]) * 
                          ( cos(theta[2] + theta[3]) * 
                            (cos(theta[1]) * Tobj(1,3) 
                             + sin(theta[1])*Tobj(2,3)) 
                            - (sin(theta[2]+theta[3])*Tobj(3,3)) ) + 
                          sin(theta[4])*(-sin(theta[1])*Tobj(1,3) 
                                         + cos(theta[1])*Tobj(2,3)) , 
                          sin(theta[2]+theta[3]) * (cos(theta[1]) * Tobj(1,3) 
                                                    + sin(theta[1])*Tobj(2,3) ) 
                          + (cos(theta[2]+theta[3])*Tobj(3,3)) );

        theta[6] = atan2( -sin(theta[4]) 
                          * ( cos(theta[2] + theta[3]) * 
                              (cos(theta[1]) * Tobj(1,1) 
                               + sin(theta[1])*Tobj(2,1)) 
                              - (sin(theta[2]+theta[3])*Tobj(3,1))) + 
                          cos(theta[4])*(-sin(theta[1])*Tobj(1,1) 
                                         + cos(theta[1])*Tobj(2,1)), 
                          -sin(theta[4]) * ( cos(theta[2] + theta[3]) * 
                                             (cos(theta[1]) * Tobj(1,2) 
                                              + sin(theta[1])*Tobj(2,2)) 
                                             - (sin(theta[2]+theta[3])*Tobj(3,2))) +
                          cos(theta[4])*(-sin(theta[1])*Tobj(1,2) 
                                         + cos(theta[1])*Tobj(2,2)) );

        qout(1) = theta[1];
        qout(2) = theta[2];
        qout(3) = theta[3];
        qout(4) = theta[4];
        qout(5) = theta[5];
        qout(6) = theta[6];

        converge = true; 
    }
    catch(std::out_of_range & e)
    {
        converge = false; 
        qout = q_actual;
    }

    qout.Release();
    return qout;
}

ReturnMatrix Robot::inv_kin_schilling(const Matrix & Tobj, bool & converge)
/*!
  @brief Analytic Schilling inverse kinematics.

  converge will be false if the desired end effector pose is outside robot range.
*/
{
    ColumnVector qout(6), q_actual;
    q_actual = get_q();

    try
    {
        Real theta[7], K=0.0, A=0.0, B=0.0, C=0.0, D=0.0, tmp=0.0, theta234 , diff1, diff2;

        if (links[6].d)
        {
            ColumnVector tmpd6(3); 
            Matrix tmp;
            tmpd6(1)=0;
            tmpd6(2)=0;
            tmpd6(3)=links[6].d;
            tmpd6 = Tobj.SubMatrix(1,3,1,3)*tmpd6;
            Tobj.SubMatrix(1,3,4,4) = Tobj.SubMatrix(1,3,4,4) - tmpd6;
        }

        theta[1] = atan2(Tobj(2,4),Tobj(1,4));
        //Calcul les deux angles possibles
        theta[0] = atan2(-Tobj(2,4),-Tobj(1,4));

        diff1 = fabs(q_actual(1)-theta[0]);
        if (diff1 > M_PI)
            diff1 = 2*M_PI - diff1;

        diff2 = fabs(q_actual(1)-theta[1]);
        if (diff2 > M_PI)
            diff2 = 2*M_PI - diff2;

        // Prend l'angle le plus proche de sa position actuel
        if (diff1 < diff2)
            theta[1] = theta[0];

        //theta2+theta3+theta4	
        theta234 = atan2( Tobj(3,3) , cos(theta[1])*Tobj(1,3) + sin(theta[1])*Tobj(2,3) );

        theta[5] = atan2( (cos(theta234)*(cos(theta[1])*Tobj(1,3) + 
                                          sin(theta[1])*Tobj(2,3)) + sin(theta234)*Tobj(3,3)),
                          (sin(theta[1])*Tobj(1,3) - cos(theta[1])*Tobj(2,3)));

        theta[6] = atan2( (-sin(theta234)*(cos(theta[1])*Tobj(1,1) + 
                                           sin(theta[1])*Tobj(2,1)) + cos(theta234)*Tobj(3,1)),
                          (-sin(theta234)*(cos(theta[1])*Tobj(1,2) + sin(theta[1])*Tobj(2,2)) + 
                           cos(theta234)*Tobj(3,2)));

        A= cos(theta[1])*Tobj(1,4) + sin(theta[1])*Tobj(2,4) - links[1].a - 
            links[4].a*cos(theta234);

        B= Tobj(3,4) - links[1].d - links[4].a*sin(theta234);

        //cos(theta3)
        K= ( A*A + B*B - (links[3].a*links[3].a) - (links[2].a*links[2].a) ) /
            ( 2*links[2].a*links[3].a );

        tmp = 1-K*K;
        if (tmp < 0)
            throw std::out_of_range("sqrt of negative number not allowed.");

        theta[3] = atan2(sqrt(tmp),K);
        theta[0] = atan2(-sqrt(tmp),K);

        diff1 = fabs(q_actual(3)-theta[0]);
        if (diff1 > M_PI)
            diff1 = 2*M_PI - diff1;

        diff2 = fabs(q_actual(3)-theta[3]);
        if (diff2 > M_PI)
            diff2 = 2*M_PI - diff2;

        // Prend l'angle le plus proche de sa position actuel
        if (diff1 < diff2)
            theta[3] = theta[0];

        C = cos(theta[3])*links[3].a + links[2].a;
        D = sin(theta[3])*links[3].a;

        theta[2] = atan2(B,A) - atan2(D,C);

        theta[4] = theta234 - theta[2] - theta[3];

        qout(1) = theta[1];
        qout(2) = theta[2];
        qout(3) = theta[3];
        qout(4) = theta[4];
        qout(5) = theta[5];
        qout(6) = theta[6];
        converge = true; 
    }
    catch(std::out_of_range & e)
    {
        converge = false;
        qout = q_actual;
    }

    qout.Release();
    return qout;
}

// ----------------- M O D I F I E D  D H  N O T A T I O N ------------------


ReturnMatrix mRobot::inv_kin(const Matrix & Tobj, const int mj)
//!  @brief Overload inv_kin function.
{
   bool converge = false;
   return inv_kin(Tobj, mj, dof, converge);
}


ReturnMatrix mRobot::inv_kin(const Matrix & Tobj, const int mj, const int endlink, bool & converge)
/*!
  @brief Inverse kinematics solutions.

  The solution is based on the analytic inverse kinematics if robot type (familly)
  is Rhino or Puma, otherwise used the numerical algoritm defined in Robot_basic
  class.
*/
{
    switch (robotType) {
	case RHINO:
	    return inv_kin_rhino(Tobj, converge);
	    break;
	case PUMA:
	    return inv_kin_puma(Tobj, converge);
	    break;
	case SCHILLING:
	    return inv_kin_schilling(Tobj, converge);
	    break;
	default:
	    return Robot_basic::inv_kin(Tobj, mj, endlink, converge);
    }
}


ReturnMatrix mRobot::inv_kin_rhino(const Matrix & Tobj, bool & converge)
/*!
  @brief Analytic Rhino inverse kinematics.

  converge will be false if the desired end effector pose is outside robot range.
*/
{
    ColumnVector qout(5), q_actual;
    q_actual = get_q();

    try
    {
        Real theta[6] , diff1, diff2, tmp,
        angle , L=0.0 , M=0.0 , K=0.0 , H=0.0 , G=0.0;

        if (links[6].d > 0)
        {
            ColumnVector tmpd6(3); 
            tmpd6(1)=0; tmpd6(2)=0; tmpd6(3)=links[6].d;
            tmpd6 = Tobj.SubMatrix(1,3,1,3)*tmpd6;
            Tobj.SubMatrix(1,3,4,4) = Tobj.SubMatrix(1,3,4,4) - tmpd6;
        }

        // Calcul les deux angles possibles
        theta[0] = atan2(Tobj(2,4),
                         Tobj(1,4));

        theta[1] = atan2(-Tobj(2,4),
                         -Tobj(1,4))  ;

        diff1 = fabs(q_actual(1)-theta[0]) ;		
        if (diff1 > M_PI)
            diff1 = 2*M_PI - diff1;

        diff2 = fabs(q_actual(1)-theta[1]);
        if (diff2 > M_PI)
            diff2 = 2*M_PI - diff2 ;

        // Prend l'angle le plus proche de sa position actuel
        if (diff1 < diff2)		   
            theta[1] = theta[0] ;

        theta[5] = atan2(sin(theta[1])*Tobj(1,1) - cos(theta[1])*Tobj(2,1), 
                         sin(theta[1])*Tobj(1,2) - cos(theta[1])*Tobj(2,2));

        // angle = theta1 +theta2 + theta3
        angle = atan2(-1*cos(theta[1])*Tobj(1,3) - sin(theta[1])*Tobj(2,3),
                      -1*Tobj(3,3));

        L = cos(theta[1])*Tobj(1,4) + 
            sin(theta[1])*Tobj(2,4) + 
            links[5].d*sin(angle) - 
            links[5].a*cos(angle);
        M = links[1].d - 
            Tobj(3,4) - 
            links[5].d*cos(angle) - 
            links[5].a*sin(angle);
        K = (L*L + M*M - links[4].a*links[4].a - 
             links[3].a*links[3].a) / 
            (2 * links[4].a * links[4].a);

        tmp = 1-K*K;
        if (tmp < 0)
            throw std::out_of_range("sqrt of negative number not allowed.");

        theta[0] = atan2( sqrt(tmp) , K );
        theta[3] = atan2( -sqrt(tmp) , K );	

        diff1 = fabs(q_actual(3)-theta[0]) ;
        if (diff1 > M_PI)
            diff1 = 2*M_PI - diff1 ;

        diff2 = fabs(q_actual(3)-theta[3]);
        if (diff2 > M_PI)
            diff2 = 2*M_PI - diff2 ;

        if (diff1 < diff2)
            theta[3] = theta[0] ;

        H = cos(theta[3]) * links[4].a + links[3].a;
        G = sin(theta[3]) * links[4].a;

        theta[2] = atan2( M , L ) - atan2( G , H );
        theta[4] = atan2( -1*cos(theta[1])*Tobj(1,3) - sin(theta[1])*Tobj(2,3) , 
                          -1*Tobj(3,3)) - theta[2] - theta[3] ;

        qout(1) = theta[1];
        qout(2) = theta[2];
        qout(3) = theta[3];
        qout(4) = theta[4];
        qout(5) = theta[5];

        converge = true;
    }
    catch(std::out_of_range & e)
    {
        converge = false; 
        qout = q_actual;
    }

    qout.Release();
    return qout;
}


ReturnMatrix mRobot::inv_kin_puma(const Matrix & Tobj, bool & converge)
/*!
  @brief Analytic Puma inverse kinematics.

  converge will be false if the desired end effector pose is outside robot range.
*/
{
    ColumnVector qout(6), q_actual;
    q_actual = get_q();

    try
    {  
        Real theta[7] , diff1, diff2, tmp,
        A = 0.0 , B = 0.0 , C = 0.0 , D =0.0, Ro = 0.0,
        H = 0.0 , L = 0.0 , M = 0.0;

        // Removed d6 component because in the Puma inverse kinematics solution 
        // d6 = 0. 
        if (links[6].d)
        {
            ColumnVector tmpd6(3); Matrix tmp;
            tmpd6(1)=0; tmpd6(2)=0; tmpd6(3)=links[6].d;
            tmpd6 = Tobj.SubMatrix(1,3,1,3)*tmpd6;
            Tobj.SubMatrix(1,3,4,4) = Tobj.SubMatrix(1,3,4,4) - tmpd6;
        }

        tmp = Tobj(2,4)*Tobj(2,4) + Tobj(1,4)*Tobj(1,4);
        if (tmp < 0)
            throw std::out_of_range("sqrt of negative number not allowed.");

        Ro = sqrt(tmp);
        D = (links[2].d+links[3].d) / Ro;

        tmp = 1-D*D;
        if (tmp < 0)
            throw std::out_of_range("sqrt of negative number not allowed.");

        //Calcul les deux angles possibles
        theta[0] =  atan2(Tobj(2,4),Tobj(1,4)) - atan2(D, sqrt(tmp));	 
        //Calcul les deux angles possibles
        theta[1] =  atan2(Tobj(2,4),Tobj(1,4)) - atan2(D , -sqrt(tmp));

        diff1 = fabs(q_actual(1)-theta[0]);
        if (diff1 > M_PI)
            diff1 = 2*M_PI - diff1;

        diff2 = fabs(q_actual(1)-theta[1]);
        if (diff2 > M_PI)
            diff2 = 2*M_PI - diff2;

        // Prend l'angle le plus proche de sa position actuel
        if (diff1 < diff2)
            theta[1] = theta[0];

        tmp = links[4].a*links[4].a + links[4].d*links[4].d;
        if (tmp < 0)
            throw std::out_of_range("sqrt of negative number not allowed.");

        Ro = sqrt(tmp);
        B = atan2(links[4].d,links[4].a);
        C = Tobj(1,4)*Tobj(1,4) + 
            Tobj(2,4)*Tobj(2,4) + 
            (Tobj(3,4)-links[1].d)*(Tobj(3,4)-links[1].d) - 
            (links[2].d + links[3].d)*(links[2].d + links[3].d) - 
            links[3].a*links[3].a - 
            links[4].a*links[4].a - 
            links[4].d*links[4].d; 
        A = C / (2*links[3].a);

        tmp = 1-A/Ro*A/Ro;
        if (tmp < 0)
            throw std::out_of_range("sqrt of negative number not allowed.");

        theta[0] = atan2(sqrt(tmp) , A/Ro) + B;
        theta[3] = atan2(-sqrt(tmp) , A/Ro) + B;

        diff1 = fabs(q_actual(3)-theta[0]);
        if (diff1 > M_PI)
            diff1 = 2*M_PI - diff1 ;

        diff2 = fabs(q_actual(3)-theta[3]);
        if (diff2 > M_PI)
            diff2 = 2*M_PI - diff2;

        //Prend l'angle le plus proche de sa position actuel
        if (diff1 < diff2)
            theta[3] = theta[0];

        H = cos(theta[1])*Tobj(1,4) + sin(theta[1])*Tobj(2,4);
        L = sin(theta[3])*links[4].d + cos(theta[3])*links[4].a + links[3].a;
        M = cos(theta[3])*links[4].d - sin(theta[3])*links[4].a;

        theta[2] = atan2( M , L ) - atan2(Tobj(3,4)-links[1].d , H );

        theta[0] = atan2( -sin(theta[1])*Tobj(1,3) + cos(theta[1])*Tobj(2,3) , 
                          cos(theta[2] + theta[3]) * 
                          (cos(theta[1]) * Tobj(1,3) + sin(theta[1])*Tobj(2,3)) 
                          - (sin(theta[2]+theta[3])*Tobj(3,3)) );

        theta[4] = atan2(-1*(-sin(theta[1])*Tobj(1,3) + cos(theta[1])*Tobj(2,3)), 
                         -cos(theta[2] + theta[3]) * 
                         (cos(theta[1]) * Tobj(1,3) + sin(theta[1])*Tobj(2,3)) 
                         + (sin(theta[2]+theta[3])*Tobj(3,3)) );

        diff1 = fabs(q_actual(4)-theta[0]);
        if (diff1 > M_PI)
            diff1 = 2*M_PI - diff1;

        diff2 = fabs(q_actual(4)-theta[4]);
        if (diff2 > M_PI)
            diff2 = 2*M_PI - diff2;

        // Prend l'angle le plus proche de sa position actuel
        if (diff1 < diff2)
            theta[4] = theta[0];

        theta[5] = atan2( cos(theta[4]) * 
                          ( cos(theta[2] + theta[3]) * 
                            (cos(theta[1]) * Tobj(1,3) 
                             + sin(theta[1])*Tobj(2,3)) 
                            - (sin(theta[2]+theta[3])*Tobj(3,3)) ) + 
                          sin(theta[4])*(-sin(theta[1])*Tobj(1,3) 
                                         + cos(theta[1])*Tobj(2,3)) , 
                          sin(theta[2]+theta[3]) * (cos(theta[1]) * Tobj(1,3) 
                                                    + sin(theta[1])*Tobj(2,3) ) 
                          + (cos(theta[2]+theta[3])*Tobj(3,3)) );

        theta[6] = atan2( -sin(theta[4]) 
                          * ( cos(theta[2] + theta[3]) * 
                              (cos(theta[1]) * Tobj(1,1) 
                               + sin(theta[1])*Tobj(2,1)) 
                              - (sin(theta[2]+theta[3])*Tobj(3,1))) + 
                          cos(theta[4])*(-sin(theta[1])*Tobj(1,1) 
                                         + cos(theta[1])*Tobj(2,1)), 
                          -sin(theta[4]) * ( cos(theta[2] + theta[3]) * 
                                             (cos(theta[1]) * Tobj(1,2) 
                                              + sin(theta[1])*Tobj(2,2)) 
                                             - (sin(theta[2]+theta[3])*Tobj(3,2))) +
                          cos(theta[4])*(-sin(theta[1])*Tobj(1,2) 
                                         + cos(theta[1])*Tobj(2,2)) );

        qout(1) = theta[1];
        qout(2) = theta[2];
        qout(3) = theta[3];
        qout(4) = theta[4];
        qout(5) = theta[5];
        qout(6) = theta[6];

        converge = true; 
    }
    catch(std::out_of_range & e)
    {
        converge = false; 
        qout = q_actual;
    }

    qout.Release();
    return qout;
}

ReturnMatrix mRobot::inv_kin_schilling(const Matrix & Tobj, bool & converge)
/*!
  @brief Analytic Schilling inverse kinematics.

  converge will be false if the desired end effector pose is outside robot range.
*/
{
    ColumnVector qout(6), q_actual;
    q_actual = get_q();
    
    try
    {
	Real theta[7], K=0.0, A=0.0, B=0.0, C=0.0, D=0.0, tmp=0.0, theta234 , diff1, diff2;

	if (links[6].d)
	{
	    ColumnVector tmpd6(3); 
	    Matrix tmp;
	    tmpd6(1)=0;
	    tmpd6(2)=0;
	    tmpd6(3)=links[6].d;
	    tmpd6 = Tobj.SubMatrix(1,3,1,3)*tmpd6;
	    Tobj.SubMatrix(1,3,4,4) = Tobj.SubMatrix(1,3,4,4) - tmpd6;
	}

	theta[1] = atan2(Tobj(2,4),Tobj(1,4));
	//Calcul les deux angles possibles
	theta[0] = atan2(-Tobj(2,4),-Tobj(1,4));

	diff1 = fabs(q_actual(1)-theta[0]);
	if (diff1 > M_PI)
	    diff1 = 2*M_PI - diff1;
		
	diff2 = fabs(q_actual(1)-theta[1]);
	if (diff2 > M_PI)
	    diff2 = 2*M_PI - diff2;

	// Prend l'angle le plus proche de sa position actuel
	if (diff1 < diff2)
	    theta[1] = theta[0];
	
	//theta2+theta3+theta4	
	theta234 = atan2( Tobj(3,3) , cos(theta[1])*Tobj(1,3) + sin(theta[1])*Tobj(2,3) );
	
	theta[5] = atan2( (cos(theta234)*(cos(theta[1])*Tobj(1,3) + 
					  sin(theta[1])*Tobj(2,3)) + sin(theta234)*Tobj(3,3)),
			  (sin(theta[1])*Tobj(1,3) - cos(theta[1])*Tobj(2,3)));

	theta[6] = atan2( (-sin(theta234)*(cos(theta[1])*Tobj(1,1) + 
					   sin(theta[1])*Tobj(2,1)) + cos(theta234)*Tobj(3,1)),
			  (-sin(theta234)*(cos(theta[1])*Tobj(1,2) + sin(theta[1])*Tobj(2,2)) + 
			   cos(theta234)*Tobj(3,2)));

	A= cos(theta[1])*Tobj(1,4) + sin(theta[1])*Tobj(2,4) - links[2].a - 
	    links[5].a*cos(theta234);

	B= Tobj(3,4) - links[1].d - links[5].a*sin(theta234);

	//cos(theta3)
	K= ( A*A + B*B - (links[4].a*links[4].a) - (links[3].a*links[3].a) ) /
	    ( 2*links[3].a*links[4].a );

	tmp = 1-K*K;
	if (tmp < 0)
	    throw std::out_of_range("sqrt of negative number not allowed.");

	theta[3] = atan2(sqrt(tmp),K);
	theta[0] = atan2(-sqrt(tmp),K);
		
	diff1 = fabs(q_actual(3)-theta[0]);
	if (diff1 > M_PI)
	    diff1 = 2*M_PI - diff1;
		
	diff2 = fabs(q_actual(3)-theta[3]);
	if (diff2 > M_PI)
	    diff2 = 2*M_PI - diff2;
		
	// Prend l'angle le plus proche de sa position actuel
	if (diff1 < diff2)
	    theta[3] = theta[0];

	C = cos(theta[3])*links[4].a + links[3].a;
	D = sin(theta[3])*links[4].a;

	theta[2] = atan2(B,A) - atan2(D,C);

	theta[4] = theta234 - theta[2] - theta[3];

	qout(1) = theta[1];
	qout(2) = theta[2];
	qout(3) = theta[3];
	qout(4) = theta[4];
	qout(5) = theta[5];
	qout(6) = theta[6];
	
	converge = true; 
    }
    catch(std::out_of_range & e)
    {
	converge = false;
	qout = q_actual;
    }

    qout.Release();
    return qout;
}


ReturnMatrix mRobot_min_para::inv_kin(const Matrix & Tobj, const int mj)
//!  @brief Overload inv_kin function.
{
   bool converge = false;
   return inv_kin(Tobj, mj, dof, converge);
}


ReturnMatrix mRobot_min_para::inv_kin(const Matrix & Tobj, const int mj, const int endlink, bool & converge)
/*!
  @brief Inverse kinematics solutions.

  The solution is based on the analytic inverse kinematics if robot type (familly)
  is Rhino or Puma, otherwise used the numerical algoritm defined in Robot_basic
  class.
*/
{
    switch (robotType) {
	case RHINO:
	    return inv_kin_rhino(Tobj, converge);
	    break;
	case PUMA:
	    return inv_kin_puma(Tobj, converge);
	    break;
	default:
	    return Robot_basic::inv_kin(Tobj, mj, endlink, converge);
    }
}


ReturnMatrix mRobot_min_para::inv_kin_rhino(const Matrix & Tobj, bool & converge)
/*!
  @brief Analytic Rhino inverse kinematics.

  converge will be false if the desired end effector pose is outside robot range.
*/
{
    ColumnVector qout(5), q_actual;
    q_actual = get_q();

    try
    {
        Real theta[6] , diff1, diff2, tmp,
        angle , L=0.0 , M=0.0 , K=0.0 , H=0.0 , G=0.0 ;

        // Calcul les deux angles possibles
        theta[0] = atan2(Tobj(2,4),
                         Tobj(1,4));

        theta[1] = atan2(-Tobj(2,4),
                         -Tobj(1,4))  ;

        diff1 = fabs(q_actual(1)-theta[0]) ;		
        if (diff1 > M_PI)
            diff1 = 2*M_PI - diff1;

        diff2 = fabs(q_actual(1)-theta[1]);
        if (diff2 > M_PI)
            diff2 = 2*M_PI - diff2 ;

        // Prend l'angle le plus proche de sa position actuel
        if (diff1 < diff2)		   
            theta[1] = theta[0] ;

        theta[5] = atan2(sin(theta[1])*Tobj(1,1) - cos(theta[1])*Tobj(2,1), 
                         sin(theta[1])*Tobj(1,2) - cos(theta[1])*Tobj(2,2));

        // angle = theta1 +theta2 + theta3
        angle = atan2(-1*cos(theta[1])*Tobj(1,3) - sin(theta[1])*Tobj(2,3),
                      -1*Tobj(3,3));

        L = cos(theta[1])*Tobj(1,4) + 
            sin(theta[1])*Tobj(2,4) + 
            links[5].d*sin(angle) - 
            links[5].a*cos(angle);
        M = links[1].d - 
            Tobj(3,4) - 
            links[5].d*cos(angle) - 
            links[5].a*sin(angle);
        K = (L*L + M*M - links[4].a*links[4].a - 
             links[3].a*links[3].a) / 
            (2 * links[4].a * links[4].a);

        tmp = 1-K*K;
        if (tmp < 0)
            throw std::out_of_range("sqrt of negative number not allowed.");

        theta[0] = atan2( sqrt(tmp) , K );
        theta[3] = atan2( -sqrt(tmp) , K );	

        diff1 = fabs(q_actual(3)-theta[0]) ;
        if (diff1 > M_PI)
            diff1 = 2*M_PI - diff1 ;

        diff2 = fabs(q_actual(3)-theta[3]);
        if (diff2 > M_PI)
            diff2 = 2*M_PI - diff2 ;

        if (diff1 < diff2)
            theta[3] = theta[0] ;

        H = cos(theta[3]) * links[4].a + links[3].a;
        G = sin(theta[3]) * links[4].a;

        theta[2] = atan2( M , L ) - atan2( G , H );
        theta[4] = atan2( -1*cos(theta[1])*Tobj(1,3) - sin(theta[1])*Tobj(2,3) , 
                          -1*Tobj(3,3)) - theta[2] - theta[3] ;

        qout(1) = theta[1];
        qout(2) = theta[2];
        qout(3) = theta[3];
        qout(4) = theta[4];
        qout(5) = theta[5];

        converge = true;
    }
    catch(std::out_of_range & e)
    {
        converge = false; 
        qout = q_actual;
    }

    qout.Release();
    return qout;
}


ReturnMatrix mRobot_min_para::inv_kin_puma(const Matrix & Tobj, bool & converge)
/*!
  @brief Analytic Puma inverse kinematics.

  converge will be false if the desired end effector pose is outside robot range.
*/
{
    ColumnVector qout(6), q_actual;
    q_actual = get_q();

    try
    {  
        Real theta[7] , diff1, diff2, tmp,
        A = 0.0 , B = 0.0 , C = 0.0 , D =0.0, Ro = 0.0,
        H = 0.0 , L = 0.0 , M = 0.0;

        // Removed d6 component because in the Puma inverse kinematics solution 
        // d6 = 0. 
        if (links[6].d > 0)
        {
            ColumnVector tmpd6(3); Matrix tmp;
            tmpd6(1)=0; tmpd6(2)=0; tmpd6(3)=links[6].d;
            tmpd6 = Tobj.SubMatrix(1,3,1,3)*tmpd6;
            Tobj.SubMatrix(1,3,4,4) = Tobj.SubMatrix(1,3,4,4) - tmpd6;
        }

        tmp = Tobj(2,4)*Tobj(2,4) + Tobj(1,4)*Tobj(1,4);
        if (tmp < 0)
            throw std::out_of_range("sqrt of negative number not allowed.");

        Ro = sqrt(tmp);
        D = (links[2].d+links[3].d) / Ro;

        tmp = 1-D*D;
        if (tmp < 0)
            throw std::out_of_range("sqrt of negative number not allowed.");

        //Calcul les deux angles possibles
        theta[0] =  atan2(Tobj(2,4),Tobj(1,4)) - atan2(D, sqrt(tmp));	 
        //Calcul les deux angles possibles
        theta[1] =  atan2(Tobj(2,4),Tobj(1,4)) - atan2(D , -sqrt(tmp));

        diff1 = fabs(q_actual(1)-theta[0]);
        if (diff1 > M_PI)
            diff1 = 2*M_PI - diff1;

        diff2 = fabs(q_actual(1)-theta[1]);
        if (diff2 > M_PI)
            diff2 = 2*M_PI - diff2;

        // Prend l'angle le plus proche de sa position actuel
        if (diff1 < diff2)
            theta[1] = theta[0];

        tmp = links[4].a*links[4].a + links[4].d*links[4].d;
        if (tmp < 0)
            throw std::out_of_range("sqrt of negative number not allowed.");

        Ro = sqrt(tmp);
        B = atan2(links[4].d,links[4].a);
        C = Tobj(1,4)*Tobj(1,4) + 
            Tobj(2,4)*Tobj(2,4) + 
            (Tobj(3,4)-links[1].d)*(Tobj(3,4)-links[1].d) - 
            (links[2].d + links[3].d)*(links[2].d + links[3].d) - 
            links[3].a*links[3].a - 
            links[4].a*links[4].a - 
            links[4].d*links[4].d; 
        A = C / (2*links[3].a);

        tmp = 1-A/Ro*A/Ro;
        if (tmp < 0)
            throw std::out_of_range("sqrt of negative number not allowed.");

        theta[0] = atan2(sqrt(tmp) , A/Ro) + B;
        theta[3] = atan2(-sqrt(tmp) , A/Ro) + B;

        diff1 = fabs(q_actual(3)-theta[0]);
        if (diff1 > M_PI)
            diff1 = 2*M_PI - diff1 ;

        diff2 = fabs(q_actual(3)-theta[3]);
        if (diff2 > M_PI)
            diff2 = 2*M_PI - diff2;

        //Prend l'angle le plus proche de sa position actuel
        if (diff1 < diff2)
            theta[3] = theta[0];

        H = cos(theta[1])*Tobj(1,4) + sin(theta[1])*Tobj(2,4);
        L = sin(theta[3])*links[4].d + cos(theta[3])*links[4].a + links[3].a;
        M = cos(theta[3])*links[4].d - sin(theta[3])*links[4].a;

        theta[2] = atan2( M , L ) - atan2(Tobj(3,4)-links[1].d , H );

        theta[0] = atan2( -sin(theta[1])*Tobj(1,3) + cos(theta[1])*Tobj(2,3) , 
                          cos(theta[2] + theta[3]) * 
                          (cos(theta[1]) * Tobj(1,3) + sin(theta[1])*Tobj(2,3)) 
                          - (sin(theta[2]+theta[3])*Tobj(3,3)) );

        theta[4] = atan2(-1*(-sin(theta[1])*Tobj(1,3) + cos(theta[1])*Tobj(2,3)), 
                         -cos(theta[2] + theta[3]) * 
                         (cos(theta[1]) * Tobj(1,3) + sin(theta[1])*Tobj(2,3)) 
                         + (sin(theta[2]+theta[3])*Tobj(3,3)) );

        diff1 = fabs(q_actual(4)-theta[0]);
        if (diff1 > M_PI)
            diff1 = 2*M_PI - diff1;

        diff2 = fabs(q_actual(4)-theta[4]);
        if (diff2 > M_PI)
            diff2 = 2*M_PI - diff2;

        // Prend l'angle le plus proche de sa position actuel
        if (diff1 < diff2)
            theta[4] = theta[0];

        theta[5] = atan2( cos(theta[4]) * 
                          ( cos(theta[2] + theta[3]) * 
                            (cos(theta[1]) * Tobj(1,3) 
                             + sin(theta[1])*Tobj(2,3)) 
                            - (sin(theta[2]+theta[3])*Tobj(3,3)) ) + 
                          sin(theta[4])*(-sin(theta[1])*Tobj(1,3) 
                                         + cos(theta[1])*Tobj(2,3)) , 
                          sin(theta[2]+theta[3]) * (cos(theta[1]) * Tobj(1,3) 
                                                    + sin(theta[1])*Tobj(2,3) ) 
                          + (cos(theta[2]+theta[3])*Tobj(3,3)) );

        theta[6] = atan2( -sin(theta[4]) 
                          * ( cos(theta[2] + theta[3]) * 
                              (cos(theta[1]) * Tobj(1,1) 
                               + sin(theta[1])*Tobj(2,1)) 
                              - (sin(theta[2]+theta[3])*Tobj(3,1))) + 
                          cos(theta[4])*(-sin(theta[1])*Tobj(1,1) 
                                         + cos(theta[1])*Tobj(2,1)), 
                          -sin(theta[4]) * ( cos(theta[2] + theta[3]) * 
                                             (cos(theta[1]) * Tobj(1,2) 
                                              + sin(theta[1])*Tobj(2,2)) 
                                             - (sin(theta[2]+theta[3])*Tobj(3,2))) +
                          cos(theta[4])*(-sin(theta[1])*Tobj(1,2) 
                                         + cos(theta[1])*Tobj(2,2)) );

        qout(1) = theta[1];
        qout(2) = theta[2];
        qout(3) = theta[3];
        qout(4) = theta[4];
        qout(5) = theta[5];
        qout(6) = theta[6];

        converge = true; 
    }
    catch(std::out_of_range & e)
    {
        converge = false; 
        qout = q_actual;
    }

    qout.Release();
    return qout;
}


ReturnMatrix mRobot_min_para::inv_kin_schilling(const Matrix & Tobj, bool & converge)
/*!
  @brief Analytic Schilling inverse kinematics.

  converge will be false if the desired end effector pose is outside robot range.
*/
{
    ColumnVector qout(6), q_actual;
    q_actual = get_q();

    try
    {
        Real theta[7], K=0.0, A=0.0, B=0.0, C=0.0, D=0.0, tmp=0.0, theta234 , diff1, diff2;

        if (links[6].d)
        {
            ColumnVector tmpd6(3); 
            Matrix tmp;
            tmpd6(1)=0;
            tmpd6(2)=0;
            tmpd6(3)=links[6].d;
            tmpd6 = Tobj.SubMatrix(1,3,1,3)*tmpd6;
            Tobj.SubMatrix(1,3,4,4) = Tobj.SubMatrix(1,3,4,4) - tmpd6;
        }

        theta[1] = atan2(Tobj(2,4),Tobj(1,4));
        //Calcul les deux angles possibles
        theta[0] = atan2(-Tobj(2,4),-Tobj(1,4));

        diff1 = fabs(q_actual(1)-theta[0]);
        if (diff1 > M_PI)
            diff1 = 2*M_PI - diff1;

        diff2 = fabs(q_actual(1)-theta[1]);
        if (diff2 > M_PI)
            diff2 = 2*M_PI - diff2;

        // Prend l'angle le plus proche de sa position actuel
        if (diff1 < diff2)
            theta[1] = theta[0];

        //theta2+theta3+theta4	
        theta234 = atan2( Tobj(3,3) , cos(theta[1])*Tobj(1,3) + sin(theta[1])*Tobj(2,3) );

        theta[5] = atan2( (cos(theta234)*(cos(theta[1])*Tobj(1,3) + 
                                          sin(theta[1])*Tobj(2,3)) + sin(theta234)*Tobj(3,3)),
                          (sin(theta[1])*Tobj(1,3) - cos(theta[1])*Tobj(2,3)));

        theta[6] = atan2( (-sin(theta234)*(cos(theta[1])*Tobj(1,1) + 
                                           sin(theta[1])*Tobj(2,1)) + cos(theta234)*Tobj(3,1)),
                          (-sin(theta234)*(cos(theta[1])*Tobj(1,2) + sin(theta[1])*Tobj(2,2)) + 
                           cos(theta234)*Tobj(3,2)));

        A= cos(theta[1])*Tobj(1,4) + sin(theta[1])*Tobj(2,4) - links[2].a - 
            links[5].a*cos(theta234);

        B= Tobj(3,4) - links[1].d - links[5].a*sin(theta234);

        //cos(theta3)
        K= ( A*A + B*B - (links[4].a*links[4].a) - (links[3].a*links[3].a) ) /
            ( 2*links[3].a*links[4].a );

        tmp = 1-K*K;
        if (tmp < 0)
            throw std::out_of_range("sqrt of negative number not allowed.");

        theta[3] = atan2(sqrt(tmp),K);
        theta[0] = atan2(-sqrt(tmp),K);

        diff1 = fabs(q_actual(3)-theta[0]);
        if (diff1 > M_PI)
            diff1 = 2*M_PI - diff1;

        diff2 = fabs(q_actual(3)-theta[3]);
        if (diff2 > M_PI)
            diff2 = 2*M_PI - diff2;

        // Prend l'angle le plus proche de sa position actuel
        if (diff1 < diff2)
            theta[3] = theta[0];

        C = cos(theta[3])*links[4].a + links[3].a;
        D = sin(theta[3])*links[4].a;

        theta[2] = atan2(B,A) - atan2(D,C);

        theta[4] = theta234 - theta[2] - theta[3];

        qout(1) = theta[1];
        qout(2) = theta[2];
        qout(3) = theta[3];
        qout(4) = theta[4];
        qout(5) = theta[5];
        qout(6) = theta[6];

        converge = true; 
    }
    catch(std::out_of_range & e)
    {
        converge = false;
        qout = q_actual;
    }

    qout.Release();
    return qout;
}


#ifdef use_namespace
}
#endif
