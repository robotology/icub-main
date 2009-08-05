/*
ROBOOP -- A robotics object oriented package in C++
Copyright (C) 1996-2004  Richard Gourdeau  and Etienne Lachance

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

2004/08/13: Etienne Lachance
    -Modified robot initialisation matrix.
-------------------------------------------------------------------------------
*/

/*! 
  @file rtest.cpp
  @brief A test file.

  Compares results with Peter Corke MATLAB toolbox
*/

//! @brief RCS/CVS version.
static const char rcsid[] = "$Id: rtest.cpp,v 1.1 2007/07/24 16:03:10 amaldo Exp $";

#include "robot.h"
#include "quaternion.h"
#include "precisio.h"
#include <fstream>

#ifdef use_namespace
using namespace ROBOOP;
#endif

const Real PUMA560_data_DH[] =
  {0, 0, 0, 0, M_PI/2.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.35, 0, 0, 0, 
   0, 0, 0, 0.4318, 0, 0, 0, 0, 17.4, -0.3638, 0.006, 0.2275, 0.13, 0, 0, 0.524, 0, 0.539, 0, 
   0, 0, 0.15005, 0.0203, -M_PI/2.0, 0, 0, 0, 4.8, -0.0203, -0.0141, 0.07, 0.066, 0, 0, 0.086, 0, 0.0125, 0,
   0, 0, 0.4318, 0.0, M_PI/2.0, 0, 0, 0, 0.82, 0, 0.019, 0, 0.0018, 0, 0, 0.0013, 0, 0.0018, 0, 
   0, 0, 0, 0.0, -M_PI/2.0, 0, 0, 0, 0.34, 0.0, 0.0, 0.0, 0.0003, 0.0, 0.0, 0.0004, 0.0, 0.0003, 0, 
   0, 0, 0, 0, 0, 0, 0, 0, 0.09, 0.0, 0.0, 0.032, 0.00015, 0.0, 0.0, 0.00015, 0.0, 0.00004, 0};
Real PUMA560_data_mDH[] =
   //joint_type, theta, d, a, alpha
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.35, 0,
 0, 0, 0, 0.0, -M_PI/2, 0, 0, 0, 17.4, 0.068, 0.006, -0.016, 0.13, 0, 0, 0.524, 0, 0.539, 0,
 0, 0, -0.15005, 0.4318, 0, 0, 0, 0, 4.8, 0, -0.070, 0.014, 0.066, 0, 0, 0.0125, 0, 0.066, 0,
 0, 0, -0.4318, 0.0203, -M_PI/2.0, 0, 0, 0, 0.82, 0.0, 0.0, -0.019, 0.0018, 0, 0, 0.0018, 0, 0.0013, 0,
 0, 0, 0, 0, M_PI/2.0, 0, 0, 0, 0.34, 0, 0, 0.0, 0.0003, 0, 0, 0.0003, 0, 0.0004, 0,
 0, 0, 0, 0, -M_PI/2, 0, 0, 0, 0.09, 0, 0, 0.032, 0.00015, 0, 0, 0.00015, 0, 0.00004, 0};
const Real PUMA560_motor[] =
   {200e-6, -62.6111, 1.48e-3, 0, // using + and - directions average
    200e-6, 107.815, .817e-3, 0,
    200e-6, -53.7063, 1.38e-3, 0,
    33e-6,  76.0364, 71.2e-6, 0,
    33e-6,  71.923,  82.6e-6, 0,
    33e-6,  76.686,  36.7e-6, 0};

const Real STANFORD_data_DH[] =
  {0.0, 0.0, 0.4120, 0.0, -M_PI/2, 0,0,0,9.29, 0.0, 0.0175, -0.1105, 0.276, 0.0, 0, 0.255, 0.0, 0.071,0,0,0,0,0,
   0.0, 0.0, 0.1540, 0.0, M_PI/2.0, 0,0,0,5.01, 0.0, -1.054, 0.0, 0.108, 0.0, 0.0, 0.018, 0.0, 0.1,0,0,0,0,0,
   1.0, -M_PI/2.0, 0.0, 0.0, 0.0, 0,0,0,4.25, 0.0, 0.0, -6.447, 2.51, 0.0, 0.0, 2.51, 0.0, 0.006,0,0,0,0,0,
   0.0, 0.0, 0.0, 0.0, -M_PI/2.0, 0,0,0,1.08, 0.0, 0.092, -0.054, 0.002, 0.0, 0.0, 0.001, 0.0, 0.001,0,0,0,0,0,
   0.0, 0.0, 0.0, 0.0,  M_PI/2.0, 0,0,0,0.63, 0.0, 0.0, 0.566, 0.003, 0.0, 0.0, 0.003, 0.0, 0.0004,0,0,0,0,0,
   0.0, 0.0, 0.2630, 0.0, 0.0, 0,0,0,0.51, 0.0, 0.0, 1.5540, 0.013, 0.0, 0.0, 0.013, 0.0, 0.0003,0,0,0,0,0};

int main(void)
{
   int i;
   std::ifstream infile;
   infile.open("source/test.txt");
   if(!infile) {
      cerr << "Cannot open file:";
      cerr << "source/test.txt\n";
      exit (1);
   }
   Matrix Test(4,4), p1(3,1);
   Real a, eps = FloatingPointPrecision::Epsilon(); /* floating point eps */

   cout << "=====================================================\n";
   cout << " ROBOOP -- A robotics object oriented package in C++ \n";;
   cout << " TEST program \n";
   cout << "=====================================================\n";
   cout << "\n";
   cout << "Machine epsilon = " << eps << endl;
   eps *= 150.0;

   cout << "Testing translation : ";
   Real ott[] = {1.0,2.0,3.0};
   p1 << ott;
   for(i = 1; i <= 4; i++) {
      for(int j = 1; j <= 4; j++) {
         infile >> Test(i,j);
      }
   }
   a = (Test-trans(p1)).MaximumAbsoluteValue();
   if(a > eps) {
      cout << "Erreur = " << a << endl;
   } else {
      cout << "Ok" << endl;
   }

   cout << "Testing rotx : ";
   for(i = 1; i <= 4; i++) {
      for(int j = 1; j <= 4; j++) {
         infile >> Test(i,j);
      }
   }
   a = (Test-rotx(M_PI/6)).MaximumAbsoluteValue();
   if(a > eps) {
      cout << "Erreur = " << a << endl;
   } else {
      cout << "Ok" << endl;
   }

   cout << "Testing roty : ";
   for(i = 1; i <= 4; i++) {
      for(int j = 1; j <= 4; j++) {
         infile >> Test(i,j);
      }
   }
   a = (Test-roty(M_PI/6)).MaximumAbsoluteValue();
   if(a > eps) {
      cout << "Erreur = " << a << endl;
   } else {
      cout << "Ok" << endl;
   }

   cout << "Testing rotz : ";
   for(i = 1; i <= 4; i++) {
      for(int j = 1; j <= 4; j++) {
         infile >> Test(i,j);
      }
   }
   a = (Test-rotz(M_PI/6)).MaximumAbsoluteValue();
   if(a > eps) {
      cout << "Erreur = " << a << endl;
   } else {
      cout << "Ok" << endl;
   }

   cout << "Testing roll-pitch-yaw : ";
   for(i = 1; i <= 4; i++) {
      for(int j = 1; j <= 4; j++) {
         infile >> Test(i,j);
      }
   }
   a = (Test-rpy(p1)).MaximumAbsoluteValue();
   if(a > eps) {
      cout << "Erreur = " << a << endl;
   } else {
      cout << "Ok" << endl;
   }

   cout << "Testing rotation around vector : ";
   for(i = 1; i <= 4; i++) {
      for(int j = 1; j <= 4; j++) {
         infile >> Test(i,j);
      }
   }
   a = (Test-rotk(M_PI/2,p1)).MaximumAbsoluteValue();
   if(a > eps) {
      cout << "Erreur = " << a << endl;
   } else {
      cout << "Ok" << endl;
   }

   // R(z) with angle=pi/4.
   cout << "Testing quaternion to rotation matrix : ";
   for(i = 1; i <= 3; i++) {
      for(int j = 1; j <= 3; j++) {
         infile >> Test(i,j);
      }
   }
   {
      //     quaternion from angle + vector
      ColumnVector v(3); v(1)=v(2)=0.0; v(3)=1.0;
      Quaternion q(M_PI/4, v);
      a = (Test.SubMatrix(1,3,1,3)-q.R()).MaximumAbsoluteValue();
      if(a > eps) {
         cout << "Erreur = " << a << endl;
      } else {
         cout << "Ok" << endl;
      }
   }

   cout << "Testing quaternion to rotation matrix : ";
   for(i = 1; i <= 3; i++) {
      for(int j = 1; j <= 3; j++) {
         infile >> Test(i,j);
      }
   }

   {
      // quaternion from 4 parameters
      Quaternion q(M_PI/4, M_PI/6, M_PI/8, 1);
      q.unit();
      a = (Test.SubMatrix(1,3,1,3)-q.R()).MaximumAbsoluteValue();
      if(a > eps) {
         cout << "Erreur = " << a << endl;
      } else {
         cout << "Ok" << endl;
      }

      cout << "Testing quaternion to transformation matrix : ";
      for(i = 1; i <= 4; i++) {
         for(int j = 1; j <= 4; j++) {
            infile >> Test(i,j);
         }
      }
      a = (Test-q.T()).MaximumAbsoluteValue();
      if(a > eps) {
         cout << "Erreur = " << a << endl;
      } else {
         cout << "Ok" << endl;
      }

      cout << "Testing rotation matrix to quaternion : ";
      ColumnVector quat(4);
      Quaternion qq(q.R());
      Test = Matrix(4,1);
      for(i = 1; i <= 4; i++) {
         infile >> Test(i,1);
      }
      quat(1) = qq.s();
      quat.SubMatrix(2,4,1,1) = qq.v();
      a = (Test-quat).MaximumAbsoluteValue();
      if(a > eps) {
         cout << "Erreur = " << a << endl;
      } else {
         cout << "Ok" << endl;
      }

      cout << "Testing transformation matrix to quaternion : ";
      qq = Quaternion(q.T());
      Test = Matrix(4,1);
      for(i = 1; i <= 4; i++) {
         infile >> Test(i,1);
      }
      quat(1) = qq.s();
      quat.SubMatrix(2,4,1,1) = qq.v();
      a = (Test-quat).MaximumAbsoluteValue();
      if(a > eps) {
         cout << "Erreur = " << a << endl;
      } else {
         cout << "Ok" << endl;
      }
   }


   // ---------------------- R O B O T S -------------------------------------
   Robot robot_DH;
   mRobot robot_mDH;
   Matrix initrob;
   Matrix initrobm;
   short dof;

   ColumnVector qr, q, qd, qdd;
   ColumnVector Fext(3), Next(3);

   // Puma 560 in DH notation without motor
   cout << "Testing Puma 560 (DH) forward kinematic : ";
   Test = Matrix(4,4);
   for(i = 1; i <= 4; i++) {
      for(int j = 1; j <= 4; j++) {
         infile >> Test(i,j);
      }
   }
   initrob = Matrix(6,19);
   initrobm = Matrix(6,4);
   initrob << PUMA560_data_DH;
   initrobm << PUMA560_motor;
   robot_DH = Robot(initrob, initrobm);
   dof = robot_DH.get_dof();
   qr = ColumnVector(dof);
   qr = M_PI/4.0;
   robot_DH.set_q(qr);
   a = (Test-robot_DH.kine()).MaximumAbsoluteValue();
   if(a > eps) {
      cout << "Erreur = " << a << endl;
   } else {
      cout << "Ok" << endl;
   }

   cout << "Testing Puma 560 (DH) jacobian in base frame : ";
   Test = Matrix(6,6);
   for(i = 1; i <= 6; i++) {
      for(int j = 1; j <= 6; j++) {
         infile >> Test(i,j);
      }
   }
   a = (Test-robot_DH.jacobian()).MaximumAbsoluteValue();
   if(a > eps) {
      cout << "Erreur = " << a << endl;
   } else {
      cout << "Ok" << endl;
   }

   cout << "Testing Puma 560 (DH) jacobian in tool frame : ";
   for(i = 1; i <= 6; i++) {
      for(int j = 1; j <= 6; j++) {
         infile >> Test(i,j);
      }
   }
   a = (Test-robot_DH.jacobian(dof)).MaximumAbsoluteValue();
   if(a > eps) {
      cout << "Erreur = " << a << endl;
   } else {
      cout << "Ok" << endl;
   }

   initrobm = Matrix(6,4);
   initrobm = 0.0;
   robot_DH = Robot(initrob,initrobm);
   Test = Matrix(dof,1); Test = 0;
   cout << "Testing Puma 560 (DH) torque : ";
   for(i = 1; i <= dof; i++) {
      infile >> Test(i,1);
   }

   qd = qr;
   qdd = qr;
   a = (Test-robot_DH.torque(qr, qd, qdd)).MaximumAbsoluteValue();
   if(a > eps) {
      cout << "Erreur = " << a << endl;
   } else {
      cout << "Ok" << endl;
   }

   cout << "Testing Puma 560 (DH) inertia : ";
   Test = Matrix(6,6);
   for(i = 1; i <= 6; i++) {
      for(int j = 1; j <= 6; j++){
         infile >> Test(i,j);
      }
   }
   a = (Test-robot_DH.inertia(qr)).MaximumAbsoluteValue();
   if(a > eps) {
      cout << "Erreur = " << a << endl;
   } else {
      cout << "Ok" << endl;
   }

   initrobm = Matrix(6,4);
   initrobm << PUMA560_motor;
   robot_DH = Robot(initrob,initrobm);
   dof = robot_DH.get_dof();
   
   cout << "Testing Puma 560 (DH) motor, torque : ";
   Test = Matrix(dof,1);
   for(i = 1; i <= dof; i++) {
      infile >> Test(i,1);
   }
   a = (Test-robot_DH.torque(qr, qd, qdd)).MaximumAbsoluteValue();
   if(a > eps) {
      cout << "Erreur = " << a << endl;
   } else {
      cout << "Ok" << endl;
   }

   // Stanford in DH notation
   cout << "Testing Stanford (DH) forward kinematic : ";
   Test = Matrix(4,4);
   for(i = 1; i <= 4; i++) {
      for(int j = 1; j <= 4; j++) {
         infile >> Test(i,j);
      }
   }
   initrob = Matrix(6,23);
   initrob << STANFORD_data_DH;
   robot_DH = Robot(initrob);
   dof = robot_DH.get_dof();
   qr = ColumnVector(dof);
   qr = M_PI/4.0;
   robot_DH.set_q(qr);
   a = (Test-robot_DH.kine()).MaximumAbsoluteValue();
   if(a > eps) {
      cout << "Erreur = " << a << endl;
   } else {
      cout << "Ok" << endl;
   }

   cout << "Testing Stanford (DH) jacobian in base frame : ";
   Test = Matrix(6,6);
   for(i = 1; i <= 6; i++) {
      for(int j = 1; j <= 6; j++) {
         infile >> Test(i,j);
      }
   }
   a = (Test-robot_DH.jacobian()).MaximumAbsoluteValue();
   if(a > eps) {
      cout << "Erreur = " << a << endl;
   } else {
      cout << "Ok" << endl;
   }

   cout << "Testing Stanford (DH) jacobian in tool frame : ";
   for(i = 1; i <= 6; i++) {
      for(int j = 1; j <= 6; j++) {
         infile >> Test(i,j);
      }
   }
   a = (Test-robot_DH.jacobian(dof)).MaximumAbsoluteValue();
   if(a > eps) {
      cout << "Erreur = " << a << endl;
   } else {
      cout << "Ok" << endl;
   }

   Test = Matrix(dof,1); Test = 0;
   cout << "Testing Stanford (DH) torque : ";
   for(i = 1; i <= dof; i++) {
      infile >> Test(i,1);
   }
   a = (Test-robot_DH.torque(qr, qd, qdd)).MaximumAbsoluteValue();
   if(a > eps) {
      cout << "Erreur = " << a << endl;
   } else {
      cout << "Ok" << endl;
   }

   cout << "Testing Stanford (DH) torque with load on link n: ";
   Fext(1)=10; Fext(2)=5; Fext(3)=7;
   Next(1)=11; Next(2)=22; Next(3)=33;
   for(i = 1; i <= dof; i++) {
      infile >> Test(i,1);
   }
   a = (Test-robot_DH.torque(qr, qd, qdd, Fext, Next)).MaximumAbsoluteValue();
   if(a > eps) {
      cout << "Erreur = " << a << endl;
   } else {
      cout << "Ok" << endl;
   }

   cout << "Testing Stanford (DH) inertia : ";
   Test = Matrix(6,6); Test = 0;
   for(i = 1; i <= 6; i++) {
      for(int j = 1; j <= 6; j++){
         infile >> Test(i,j);
      }
   }
   a = (Test-robot_DH.inertia(qr)).MaximumAbsoluteValue();
   if(a > eps) {
      cout << "Erreur = " << a << endl;
   } else {
      cout << "Ok" << endl;
   }

   Test = Matrix(4,4);

   // ATTENTION J'AI CHANGE LES PARAMETRES DH DANS PUMA560AKD.M, j'ai ecris a P. Corke
   // Puma 560 DH modified
   cout << "Testing Puma 560 (mDH) forward kinematic : ";
   for(i = 1; i <= 4; i++) {
      for(int j = 1; j <= 4; j++) {
         infile >> Test(i,j);
      }
   }
   initrob = Matrix(6,19);
   initrob << PUMA560_data_mDH;
   initrobm = Matrix(6,4);
   initrobm = 0.0;
   robot_mDH = mRobot(initrob, initrobm);
   dof = robot_mDH.get_dof();
   qr = ColumnVector(dof);
   qr = M_PI/4.0;
   robot_mDH.set_q(qr);
   a = (Test-robot_mDH.kine()).MaximumAbsoluteValue();
   if(a > eps) {
      cout << "Erreur = " << a << endl;
   } else {
      cout << "Ok" << endl;
   }

   // faut revoir jacobian pour dernier link
   cout << "Testing Puma 560 (mDH) jacobian in base frame : ";
   Test = Matrix(6,6);
   for(i = 1; i <= 6; i++) {
      for(int j = 1; j <= 6; j++) {
         infile >> Test(i,j);
      }
   }
   a = (Test-robot_mDH.jacobian()).MaximumAbsoluteValue();
   if(a > eps) {
      cout << "Erreur = " << a << endl;
   } else {
      cout << "Ok" << endl;
   }

   cout << "Testing Puma 560 (mDH) jacobian in tool frame : ";
   for(i = 1; i <= 6; i++) {
      for(int j = 1; j <= 6; j++) {
         infile >> Test(i,j);
      }
   }
   a = (Test-robot_mDH.jacobian(dof)).MaximumAbsoluteValue();
   if(a > eps) {
      cout << "Erreur = " << a << endl;
   } else {
      cout << "Ok" << endl;
   }

   initrobm = Matrix(6,4);
   initrobm = 0.0;
   robot_mDH = mRobot(initrob,initrobm);

   cout << "Testing Puma 560 (mDH) torque : ";
   Test = Matrix(dof,1);
   for(i = 1; i <= dof; i++) {
      infile >> Test(i,1);
   }
   a = (Test-robot_mDH.torque(qr, qd, qdd)).MaximumAbsoluteValue();
   if(a > eps) {
      cout << "Erreur = " << a << endl;
   } else {
      cout << "Ok" << endl;
   }

   cout << "Testing Puma 560 (mDH) inertia : ";
   Test = Matrix(6,6); Test = 0;
   for(i = 1; i <= 6; i++) {
      for(int j = 1; j <= 6; j++){
         infile >> Test(i,j);
      }
   }
   a = (Test-robot_mDH.inertia(qr)).MaximumAbsoluteValue();
   if(a > eps) {
      cout << "Erreur = " << a << endl;
   } else {
      cout << "Ok" << endl;
   }

   cout << "Testing Puma 560 (mDH) motor, torque : ";
   initrobm = Matrix(6,4);
   initrobm << PUMA560_motor;
   robot_mDH = mRobot(initrob,initrobm);
   Test = Matrix(dof,1);
   for(i = 1; i <= dof; i++) {
      infile >> Test(i,1);
   }
   a = (Test-robot_mDH.torque(qr, qd, qdd)).MaximumAbsoluteValue();
   if(a > eps) {
      cout << "Erreur = " << a << endl;
   } else {
      cout << "Ok" << endl;
   }

   cout << "Testing Puma 560 (mDH) motor, torque with load on link n: ";
   Fext(1)=10; Fext(2)=5; Fext(3)=7;
   Next(1)=11; Next(2)=22; Next(3)=33;
   for(i = 1; i <= dof; i++) {
      infile >> Test(i,1);
   }
   a = (Test-robot_mDH.torque(qr, qd, qdd, Fext, Next)).MaximumAbsoluteValue();
   if(a > eps) {
      cout << "Erreur = " << a << endl;
   } else {
      cout << "Ok" << endl;
   }

   return(0);
}
