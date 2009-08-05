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
   -Added quaternions example in homogen_demo.
   -Using function set_plot2d for gnuplot graphs. 
   -Declare vector with "dof" instead of hardcoded value.
   -Changed RobotMotor to Robot.

2003/29/04: Etienne Lachance
   -Using Robot("conf/puma560_dh.conf", "PUMA560_DH") in kinematics functions.

2004/07/01: Ethan Tira-Thompson
    -Added support for newmat's use_namespace #define, using ROBOOP namespace

2004/08/13: Etienne Lachance
    -Modified robot initialisation matrix.

2005/11/15 : Richard Gourdeau
    - Fixed error on PUMA560 without motor dynamics
-------------------------------------------------------------------------------

*/

/*! 
  @file demo.cpp
  @brief A demo file.

  Demos for homogeneous transforms, kinematics, etc.
*/

//! @brief RCS/CVS version.
static const char rcsid[] = "$Id: demo.cpp,v 1.1 2007/07/24 16:03:09 amaldo Exp $";

#include "gnugraph.h"
#include "quaternion.h"
#include "robot.h"
#include "utils.h"

#ifdef use_namespace
using namespace ROBOOP;
#endif

void homogen_demo(void);
void kinematics_demo(void);
void dynamics_demo(void);


int main(void)
{
   cout << "=====================================================\n";
   cout << " ROBOOP -- A robotics object oriented package in C++ \n";;
   cout << " DEMO program \n";
   cout << "=====================================================\n";
   cout << "\n";

   homogen_demo();
   kinematics_demo();
   dynamics_demo();

   return(0);
}


void homogen_demo(void)
{
   ColumnVector p1(3), p2(3), p3(3);
   Real ott[] = {1.0,2.0,3.0};
   Real tto[] = {3.0,2.0,1.0};

   cout << "\n";
   cout << "=====================================================\n";
   cout << "Homogeneous transformations\n";
   cout << "=====================================================\n";
   cout << "\n";
   cout << "Translation of [1;2;3]\n";
   p1 << ott;
   cout << setw(7) << setprecision(3) << trans(p1);
   cout << "\n";
   cout << "\n";
   cout << "Rotation of pi/6 around axis X\n";
   cout << setw(7) << setprecision(3) << rotx(M_PI/6);
   cout << "\n";

   cout << "\n";
   cout << "Rotation of pi/8 around axis Y\n";
   cout << setw(7) << setprecision(3) << roty(M_PI/8);
   cout << "\n";

   cout << "\n";
   cout << "Rotation of pi/3 around axis Z\n";
   cout << setw(7) << setprecision(3) << rotz(M_PI/3);
   cout << "\n";

   cout << "\n";
   cout << "Rotation of pi/3 around axis [1;2;3]\n";
   cout << setw(7) << setprecision(3) << rotk(M_PI/3,p1);
   cout << "\n";

   cout << "\n";
   cout << "Rotation of pi/6 around line [1;2;3] -- [3;2;1]\n";
   p2 << tto;
   cout << setw(7) << setprecision(3) << rotd(M_PI/3,p1,p2);
   cout << "\n";

   cout << "\n";
   cout << "Roll Pitch Yaw Rotation [1;2;3]\n";
   cout << setw(7) << setprecision(3) << rpy(p1);
   cout << "\n";

   cout << "\n";
   cout << "Euler ZXZ Rotation [3;2;1]\n";
   cout << setw(7) << setprecision(3) << eulzxz(p2);
   cout << "\n";

   cout << "\n";
   cout << "Inverse of Rotation around axis\n";
   cout << setw(7) << setprecision(3) << irotk(rotk(M_PI/3,p1));
   cout << "\n";

   cout << "\n";
   cout << "Inverse of Roll Pitch Yaw Rotation\n";
   cout << setw(7) << setprecision(3) << irpy(rpy(p1));
   cout << "\n";

   cout << "\n";
   cout << "Inverse of Euler ZXZ Rotation\n";
   cout << setw(7) << setprecision(3) << ieulzxz(eulzxz(p2));
   cout << "\n";

   cout << "\n";
   cout << "Cross product between [1;2;3] and [3;2;1]\n";
   cout << setw(7) << setprecision(3) << CrossProduct(p1,p2);
   cout << "\n";

   cout << "\n";
   cout << "Rotation matrix from quaternion\n";
   ColumnVector axis(3); axis(1)=axis(2)=0; axis(3)=1.0;
   Quaternion q(M_PI/4, axis);
   cout << setw(7) << setprecision(3) << q.R();
   cout << "\n";

   cout << "\n";
   cout << "Transformation matrix from quaternion\n";
   cout << setw(7) << setprecision(3) << q.T();

   cout << "\n";
   cout << "Quaternion from rotation matrix\n";
   Quaternion qq(q.R());
   cout << setw(7) << setprecision(3) << qq.s() << endl;
   cout << setw(7) << setprecision(3) << qq.v() << endl;
   cout << "\n";

   cout << "\n";
   cout << "Quaternion from transformation matrix\n";
   qq = Quaternion(q.T());
   cout << setw(7) << setprecision(3) << qq.s() << endl;
   cout << setw(7) << setprecision(3) << qq.v() << endl;
}

const Real RR_data[] =
  {0, 0, 0, 1.0, 0, 0, 0, 0, 2.0,-0.5, 0, 0, 0, 0, 0, 0.1666666, 0, 0.1666666, 0, 0, 0, 0, 0,
   0, 0, 0, 1.0, 0, 0, 0, 0, 1.0,-0.5, 0, 0, 0, 0, 0, 0.0833333, 0, 0.0833333, 0, 0, 0, 0, 0};
const Real RR_data_mdh[] =
  {0, 0, 0, 1.0, 0, 0, 0, 0, 2.0, 0.5, 0, 0, 0, 0, 0, 0.1666666, 0, 0.1666666, 0, 0, 0, 0, 0,
   0, 0, 0, 1.0, 0, 0, 0, 0, 1.0, 0.5, 0, 0, 0, 0, 0, 0.0833333, 0, 0.0833333, 0, 0, 0, 0, 0};
const Real RR_data_mdh_min_para[] =
  {0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0.0, 0, 0,     0, 0, 0, 0, 0.0, 1.666666, 0, 0, 0, 0, 0,
   0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0.5, 0, 0, -0.25, 0, 0, 0, 0.0, 0.3333333, 0, 0, 0, 0, 0};

const Real RP_data[] =
  {0, 0, 0, 0, -M_PI/2.0, 0, 0, 0, 2.0, 0, 0, 0.0, 1.0, 0, 0, 1.0, 0, 1.0, 0, 0, 0, 0, 0,
   1, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0,-1.0, 0.0833333, 0, 0, 0.0833333, 0, 0.0833333, 0, 0, 0, 0, 0};
const Real PUMA560_data[] =
  {0, 0, 0, 0, M_PI/2.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.35, 0, 0, 0, 
   0, 0, 0, 0.4318, 0, 0, 0, 0, 17.4, -0.3638, 0.006, 0.2275, 0.13, 0, 0, 0.524, 0, 0.539, 0, 
   0, 0, 0.15005, 0.0203, -M_PI/2.0, 0, 0, 0, 4.8, -0.0203, -0.0141, 0.07, 0.066, 0, 0, 0.086, 0, 0.0125, 0,
   0, 0, 0.4318, 0.0, M_PI/2.0, 0, 0, 0, 0.82, 0, 0.019, 0, 0.0018, 0, 0, 0.0013, 0, 0.0018, 0, 
   0, 0, 0, 0.0, -M_PI/2.0, 0, 0, 0, 0.34, 0.0, 0.0, 0.0, 0.0003, 0.0, 0.0, 0.0004, 0.0, 0.0003, 0, 
   0, 0, 0, 0, 0, 0, 0, 0, 0.09, 0.0, 0.0, 0.032, 0.00015, 0.0, 0.0, 0.00015, 0.0, 0.00004, 0};
const Real PUMA560_motor[] =
  {200e-6, -62.6111, 1.48e-3, (.395 +.435)/2, /* using + and - directions average */
   200e-6, 107.815, .817e-3, (.126 + .071)/2,
   200e-6, -53.7063, 1.38e-3, (.132	+ .105)/2,
   33e-6,   76.0364, 71.2e-6, (11.2e-3 + 16.9e-3)/2,
   33e-6,   71.923, 82.6e-6, (9.26e-3 + 14.5e-3)/2,
   33e-6,   76.686, 36.7e-6, (3.96e-3 + 10.5e-3)/2};

const Real STANFORD_data[] =
  {0.0, 0.0, 0.4120, 0.0, -M_PI/2, 0,0,0,9.29, 0.0, 0.0175, -0.1105, 0.276, 0.0, 0, 0.255, 0.0, 0.071,0,0,0,0,0,
   0.0, 0.0, 0.1540, 0.0, M_PI/2.0, 0,0,0,5.01, 0.0, -1.054, 0.0, 0.108, 0.0, 0.0, 0.018, 0.0, 0.1,0,0,0,0,0,
   1.0, -M_PI/2.0, 0.0, 0.0, 0.0, 0,0,0,4.25, 0.0, 0.0, -6.447, 2.51, 0.0, 0.0, 2.51, 0.0, 0.006,0,0,0,0,0,
   0.0, 0.0, 0.0, 0.0, -M_PI/2.0, 0,0,0,1.08, 0.0, 0.092, -0.054, 0.002, 0.0, 0.0, 0.001, 0.0, 0.001,0,0,0,0,0,
   0.0, 0.0, 0.0, 0.0,  M_PI/2.0, 0,0,0,0.63, 0.0, 0.0, 0.566, 0.003, 0.0, 0.0, 0.003, 0.0, 0.0004,0,0,0,0,0,
   0.0, 0.0, 0.2630, 0.0, 0.0, 0,0,0,0.51, 0.0, 0.0, 1.5540, 0.013, 0.0, 0.0, 0.013, 0.0, 0.0003,0,0,0,0,0};

Robot robot;
Matrix K;
ColumnVector q0;

ReturnMatrix xdot(Real t, const Matrix & x)
{
   int ndof;
   ColumnVector q, qp, qpp; /* position, velocities and accelerations */
   ColumnVector tau, dx;              /* torque and state space error */
   Matrix xd;

   ndof = robot.get_dof();                             /* get the # of dof  */
   q = x.SubMatrix(1,ndof,1,1);               /* position, velocities */
   qp = x.SubMatrix(ndof+1,2*ndof,1,1);          /* from state vector */
   qpp = ColumnVector(ndof);
   qpp = 0.0;                               /* set the vector to zero */
   //   tau = robot.torque(q0,qpp,qpp); /* compute the gravity torque */
   tau = ColumnVector(ndof);
   tau = 0.0;

   dx = (q-q0) & qp;       /* compute dx using vertical concatenation */
   tau = tau - K*dx;                           /* feedback correction */
   qpp = robot.acceleration(q, qp, tau);              /* acceleration */
   xd = qp & qpp;                          /* state vector derivative */

   xd.Release(); return xd;
}

void kinematics_demo(void)
{
   Matrix initrob(2,23), Tobj;
   ColumnVector qs, qr;
   int dof = 0;
   int i;

   cout << "\n";
   cout << "=====================================================\n";
   cout << "Robot RR kinematics\n";
   cout << "=====================================================\n";
   initrob << RR_data;
   robot = Robot(initrob);
   dof = robot.get_dof();

   cout << "\n";
   cout << "Robot D-H parameters\n";
   cout << "   type     theta      d        a      alpha\n";
   cout << setw(7) << setprecision(3) << initrob.SubMatrix(1,dof,1,5);
   cout << "\n";
   cout << "Robot joints variables\n";
   cout << setw(7) << setprecision(3) << robot.get_q();
   cout << "\n";
   cout << "Robot position\n";
   cout << setw(7) << setprecision(3) << robot.kine();
   cout << "\n";
   qr = ColumnVector(dof);
   qr = M_PI/4.0;
   robot.set_q(qr);
   cout << "Robot joints variables\n";
   cout << setw(7) << setprecision(3) << robot.get_q();
   cout << "\n";
   cout << "Robot position\n";
   cout << setw(7) << setprecision(3) << robot.kine();
   cout << "\n";
   cout << "Robot Jacobian w/r to base frame\n";
   cout << setw(7) << setprecision(3) << robot.jacobian();
   cout << "\n";
   cout << "Robot Jacobian w/r to tool frame\n";
   cout << setw(7) << setprecision(3) << robot.jacobian(qr.Nrows());
   cout << "\n";
   for (i = 1; i <= qr.Nrows(); i++) {
      cout << "Robot position partial derivative with respect to joint " << i << " \n";
      cout << setw(7) << setprecision(3) << robot.dTdqi(i);
      cout << "\n";
   }
   Tobj = robot.kine();
   qs = ColumnVector(dof);
   qs = M_PI/16.0;
   robot.set_q(qs);
   cout << "Robot inverse kinematics\n";
   cout << "  q start  q final  q real\n";
   cout << setw(7) << setprecision(3) << (qs | robot.inv_kin(Tobj) | qr);
   cout << "\n";
   cout << "\n";

   cout << "=====================================================\n";
   cout << "Robot RP kinematics\n";
   cout << "=====================================================\n";
   initrob << RP_data;
   robot = Robot(initrob);
   dof = robot.get_dof();
   cout << "\n";
   cout << "Robot D-H parameters\n";
   cout << "  type    theta     d       a     alpha\n";
   cout << setw(7) << setprecision(3) << initrob.SubMatrix(1,dof,1,5);
   cout << "\n";
   cout << "Robot joints variables\n";
   cout << setw(7) << setprecision(3) << robot.get_q();
   cout << "\n";
   cout << "Robot position\n";
   cout << setw(7) << setprecision(3) << robot.kine();
   cout << "\n";
   robot.set_q(M_PI/4.0,1);
   robot.set_q(4.0,2);
   cout << "Robot joints variables\n";
   cout << setw(7) << setprecision(3) << robot.get_q();
   cout << "\n";
   cout << "Robot position\n";
   cout << setw(7) << setprecision(3) << robot.kine();
   cout << "\n";
   cout << "Robot Jacobian w/r to base frame\n";
   cout << setw(7) << setprecision(3) << robot.jacobian();
   cout << "\n";
   qr = robot.get_q();
   cout << "Robot Jacobian w/r to tool frame\n";
   cout << setw(7) << setprecision(3) << robot.jacobian(qr.Nrows());
   cout << "\n";
   for (i = 1; i <= qr.Nrows(); i++) {
      cout << "Robot position partial derivative with respect to joint " << i << " \n";
      cout << setw(7) << setprecision(3) << robot.dTdqi(i);
      cout << "\n";
   }
   Tobj = robot.kine();
   robot.set_q(M_PI/16.0,1);
   robot.set_q(1.0,2);
   qs = robot.get_q();
   cout << "Robot inverse kinematics\n";
   cout << " q start q final q real\n";
   cout << setw(7) << setprecision(3) << (qs | robot.inv_kin(Tobj) | qr);
   cout << "\n";
   cout << "\n";

   cout << "=====================================================\n";
   cout << "Robot PUMA560 kinematics\n";
   cout << "=====================================================\n";
   robot = Robot("conf/puma560_dh.conf", "PUMA560_DH");
   dof = robot.get_dof();
   cout << "\n";
   cout << "Robot joints variables\n";
   cout << setw(7) << setprecision(3) << robot.get_q();
   cout << "\n";
   cout << "Robot position\n";
   cout << setw(7) << setprecision(3) << robot.kine();
   cout << "\n";
   qr = ColumnVector(dof);
   qr = M_PI/4.0;
   robot.set_q(qr);
   cout << "Robot joints variables\n";
   cout << setw(7) << setprecision(3) << robot.get_q();
   cout << "\n";
   cout << "Robot position\n";
   cout << setw(7) << setprecision(3) << robot.kine();
   cout << "\n";
   cout << "Robot Jacobian w/r to base frame\n";
   cout << setw(7) << setprecision(3) << robot.jacobian();
   cout << "\n";
   cout << "Robot Jacobian w/r to tool frame\n";
   cout << setw(7) << setprecision(3) << robot.jacobian(qr.Nrows());
   cout << "\n";
   for (i = 1; i <= qr.Nrows(); i++) {
      cout << "Robot position partial derivative with respect to joint " << i << " \n";
      cout << setw(7) << setprecision(3) << robot.dTdqi(i);
      cout << "\n";
   }
   Tobj = robot.kine();
   qs = ColumnVector(dof);
   qs = 1.0;
   qs(1) = M_PI/16.0;
   robot.set_q(qs);
   cout << "Robot inverse kinematics\n";
   cout << " q start q final q real\n";
   cout << setw(7) << setprecision(3) << (qs | robot.inv_kin(Tobj) | qr);
   cout << "\n";
   cout << "\n";
   cout << "=====================================================\n";
   cout << "Robot STANFORD kinematics\n";
   cout << "=====================================================\n";
   initrob = Matrix(6,23);
   initrob << STANFORD_data;
   robot = Robot(initrob);
   dof = robot.get_dof();
   cout << "\n";
   cout << "Robot D-H parameters\n";
   cout << "  type    theta     d       a     alpha\n";
   cout << setw(7) << setprecision(3) << initrob.SubMatrix(1,dof,1,5);
   cout << "\n";
   cout << "Robot joints variables\n";
   cout << setw(7) << setprecision(3) << robot.get_q();
   cout << "\n";
   cout << "Robot position\n";
   cout << setw(7) << setprecision(3) << robot.kine();
   cout << "\n";
   qr = ColumnVector(dof);
   qr = M_PI/4.0;
   robot.set_q(qr);
   cout << "Robot joints variables\n";
   cout << setw(7) << setprecision(3) << robot.get_q();
   cout << "\n";
   cout << "Robot position\n";
   cout << setw(7) << setprecision(3) << robot.kine();
   cout << "\n";
   cout << "Robot Jacobian w/r to base frame\n";
   cout << setw(7) << setprecision(3) << robot.jacobian();
   cout << "\n";
   cout << "Robot Jacobian w/r to tool frame\n";
   cout << setw(7) << setprecision(3) << robot.jacobian(qr.Nrows());
   cout << "\n";
   for (i = 1; i <= qr.Nrows(); i++) {
      cout << "Robot position partial derivative with respect to joint " << i << " \n";
      cout << setw(7) << setprecision(3) << robot.dTdqi(i);
      cout << "\n";
   }
   Tobj = robot.kine();
   qs = ColumnVector(dof);
   qs = 1.0;
   qs(1) = M_PI/16.0;
   robot.set_q(qs);
   cout << "Robot inverse kinematics\n";
   cout << " q start q final q real\n";
   cout << setw(7) << setprecision(3) << (qs | robot.inv_kin(Tobj) | qr);
   cout << "\n";
   cout << "\n";
}

void dynamics_demo(void)
{
   int nok, nbad, dof = 0;
   Matrix initrob(2,23), Tobj, xout;
   ColumnVector qs, qr;
   RowVector tout;
   int i;

   cout << "\n";
   cout << "=====================================================\n";
   cout << "Robot RR dynamics\n";
   cout << "=====================================================\n";
   initrob << RR_data;
   robot = Robot(initrob);
   dof = robot.get_dof();
   cout << "\n";
   cout << "Robot D-H parameters\n";
   cout << "  type    theta     d       a     alpha\n";
   cout << setw(7) << setprecision(3) << initrob.SubMatrix(1,dof,1,5);
   cout << "\n";
   cout << "Robot D-H inertial parameters\n";
   cout << "  mass     cx       cy      cz     Ixx     Ixy     Ixz     Iyy     Iyz     Izz\n";
   cout << setw(7) << setprecision(3) << initrob.SubMatrix(1,dof,9,18);
   cout << "\n";
   cout << "Robot joints variables\n";
   cout << setw(7) << setprecision(3) << robot.get_q();
   cout << "\n";
   cout << "Robot Inertia matrix\n";
   cout << setw(7) << setprecision(3) << robot.inertia(robot.get_q());
   cout << "\n";

   K = Matrix(dof,2*dof);
   K = 0.0;
   K(1,1) = K(2,2) = 25.0;      /* K = [25I 7.071I ] */
   K(1,3) = K(2,4) = 7.071;
   cout << "Feedback gain matrix K\n";
   cout << setw(7) << setprecision(3) << K;
   cout << "\n";
   q0 = ColumnVector(dof);
   q0 = M_PI/4.0;
   qs = ColumnVector(2*dof);
   qs = 0.0;

   cout << " time     ";
   for(i = 1; i <= dof; i++)
      cout <<"q" << i << "      ";
   for(i = 1; i <= dof; i++)
      cout <<"qp" << i << "     ";
   cout << endl;

   /*   Runge_Kutta4(xdot, qs, 0.0, 4.0, 160, tout, xout);
   replaced by adaptive step size */
   odeint(xdot, qs, 0.0, 4.0, 1e-4,0.1, 1e-12, nok, nbad, tout, xout, 0.05);
   cout << setw(7) << setprecision(3) << (tout & xout).t();
   cout << "\n";
   cout << "\n";


   set_plot2d("Robot joints position", "time (sec)", "q(i) (rad)", "q", DATAPOINTS,
              tout, xout, 1, dof);

   /* uncomment the following two lines to obtain a
      ps file of the graph */
   /*   plotposition.addcommand("set term post");
      plotposition.addcommand("set output \"demo.ps\"");*/

   set_plot2d("Robot joints velocity", "time (sec)", "qp(i) (rad/s)", "qp", DATAPOINTS,
              tout, xout, dof+1, 2*dof);


   cout << "=====================================================\n";
   cout << "Robot RP dynamics\n";
   cout << "=====================================================\n";
   initrob << RP_data;
   robot = Robot(initrob);
   dof = robot.get_dof();
   cout << "\n";
   cout << "Robot D-H parameters\n";
   cout << "  type    theta     d       a     alpha\n";
   cout << setw(7) << setprecision(3) << initrob.SubMatrix(1,dof,1,5);
   cout << "\n";
   cout << "Robot D-H inertial parameters\n";
   cout << "  mass     cx       cy      cz     Ixx     Ixy     Ixz     Iyy     Iyz     Izz\n";
   cout << setw(7) << setprecision(3) << initrob.SubMatrix(1,dof,9,18);
   cout << "\n";
   cout << "Robot joints variables\n";
   cout << setw(7) << setprecision(3) << robot.get_q();
   cout << "\n";
   cout << "Robot Inertia matrix\n";
   cout << setw(7) << setprecision(3) << robot.inertia(robot.get_q());
   cout << "\n";
   cout << "\n";

   cout << "=====================================================\n";
   cout << "Robot PUMA560 dynamics\n";
   cout << "=====================================================\n";
   initrob = Matrix(6,19);
   initrob << PUMA560_data;
   Matrix temp(6,4); 
   temp = 0; /* empty motor dynamics */
   robot = Robot((initrob | temp));
   dof = robot.get_dof();
   cout << "\n";
   cout << "Robot D-H parameters\n";
   cout << "  type    theta     d       a     alpha\n";
   cout << setw(7) << setprecision(3) << initrob.SubMatrix(1,dof,1,5);
   cout << "\n";
   cout << "Robot D-H inertial parameters\n";
   cout << "  mass     cx       cy      cz     Ixx     Ixy     Ixz     Iyy     Iyz     Izz\n";
   cout << setw(7) << setprecision(3) << initrob.SubMatrix(1,dof,9,18);
   cout << "\n";
   cout << "Robot joints variables\n";
   cout << setw(7) << setprecision(3) << robot.get_q();
   cout << "\n";
   cout << "Robot Inertia matrix\n";
   cout << setw(8) << setprecision(4) << robot.inertia(robot.get_q());
   cout << "\n";
   qs = ColumnVector(dof);
   qr = ColumnVector(dof);
   qs =0.0;
   qr =0.0;
   cout << "Robot Torque\n";
   cout << setw(8) << setprecision(4) << robot.torque(robot.get_q(),qs,qr);
   cout << "\n";
   cout << "Robot acceleration\n";
   cout << setw(8) << setprecision(4) << robot.acceleration(robot.get_q(),qs,qr);
   cout << "\n";

   cout << "\n";
   cout << "=====================================================\n";
   cout << "Robot STANFORD dynamics\n";
   cout << "=====================================================\n";
   initrob = Matrix(6,23);
   initrob << STANFORD_data;
   robot = Robot(initrob);
   dof = robot.get_dof();
   cout << "\n";
   cout << "Robot D-H parameters\n";
   cout << "  type    theta     d       a     alpha\n";
   cout << setw(7) << setprecision(3) << initrob.SubMatrix(1,dof,1,5);
   cout << "\n";
   cout << "Robot D-H inertial parameters\n";
   cout << "  mass     cx       cy      cz     Ixx     Ixy     Ixz     Iyy     Iyz     Izz\n";
   cout << setw(7) << setprecision(3) << initrob.SubMatrix(1,dof,9,18);
   cout << "\n";
   cout << "Robot joints variables\n";
   cout << setw(7) << setprecision(3) << robot.get_q();
   cout << "\n";
   cout << "Robot Inertia matrix\n";
   cout << setw(7) << setprecision(3) << robot.inertia(robot.get_q());
   cout << "\n";
   cout << "\n";

   cout << "=====================================================\n";
   cout << "Robot PUMA560 with motors dynamics\n";
   cout << "=====================================================\n";
   initrob = Matrix(6,19);
   initrob << PUMA560_data;
   Matrix initrobm = Matrix(6,4);
   initrobm << PUMA560_motor;
   robot = Robot(initrob,initrobm);
   dof =robot.get_dof();
   cout << "\n";
   cout << "Robot D-H parameters\n";
   cout << "  type    theta     d       a     alpha\n";
   cout << setw(7) << setprecision(3) << initrob.SubMatrix(1,dof,1,5);
   cout << "\n";
   cout << "Robot D-H inertial parameters\n";
   cout << "  mass     cx       cy      cz     Ixx     Ixy     Ixz     Iyy     Iyz     Izz\n";
   cout << setw(7) << setprecision(3) << initrob.SubMatrix(1,dof,9,18);
   cout << "\n";
   cout << "Robot motors inertia, gear ratio, viscous and Coulomb friction coefficients\n";
   cout << "  Im       Gr       B       Cf\n";
   cout << setw(7) << setprecision(3) << initrobm;
   cout << "\n";
   cout << "Robot joints variables\n";
   cout << setw(7) << setprecision(3) << robot.get_q();
   cout << "\n";
   cout << "Robot Inertia matrix\n";
   cout << setw(8) << setprecision(4) << robot.inertia(robot.get_q());
   cout << "\n";
   qs = ColumnVector(dof);
   qr = ColumnVector(dof);
   qs =0.0;
   qr =0.0;
   robot.set_q(qs);
   cout << "Robot Torque\n";
   cout << setw(8) << setprecision(4) << robot.torque(robot.get_q(),qs,qr);
   cout << "\n";
   cout << "Robot acceleration\n";
   cout << setw(8) << setprecision(4) << robot.acceleration(robot.get_q(),qs,qr);
   cout << "\n";
}
