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
Foundation, Inc., 59 Temple Place, Suite 330, Boston,  MA 02111-1307  USA


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
    -Added doxygen documentation.

2004/08/13: Etienne Lachance
    -Modified robot initialisation matrix.

2005/06/29: Richard Gourdeau
    -Added code for the Stewart platform by Samuel Belanger
-------------------------------------------------------------------------------
*/

/*! 
  @file bench.cpp
  @brief A benchmark file.

  Prints the time, on the console, to perform certain operations.
*/

//! @brief RCS/CVS version.
static const char rcsid[] = "$Id: bench.cpp,v 1.1 2007/07/24 16:03:09 amaldo Exp $";

#include <time.h>

#ifdef _WIN32          /* Windows 95/NT */
#ifdef __GNUG__        /* Cygnus Gnu C++ for win32*/
#include <windows.h>
#define clock() GetTickCount() /* quick fix for bug in clock() */
#endif
#endif

#include "robot.h"
#include "stewart.h"

#ifdef use_namespace
using namespace ROBOOP;
#endif

#define NTRY 2000

Real Stewart_Ini[] =
   {1.758, 2.8, -1.015, 0.225, 0.0, -0.228, 3.358, 0.05, 4.237, 0.1406, 10, 12.5, 0.5, 0.35, 0.0, 0.0, 0.0,
    1.6021, 3.07, -0.925, 0.1125, 0.1949, -0.228, 3.358, 0.05, 4.237, 0.1406, 10, 12.5, 0.5, 0.35, 0.0, 0.0, 0.0,
    -1.7580, 2.8, -1.015, -0.1125, 0.1949, -0.228, 3.358, 0.05, 4.237, 0.1406, 10, 12.5, 0.5, 0.35, 0.0, 0.0, 0.0,
    -1.6021, 3.07, -0.925, -0.225, 0.0, -0.228, 3.358, 0.05, 4.237, 0.1406, 10, 12.5, 0.5, 0.35, 0.0, 0.0, 0.0,
    0.0, 2.8, 2.03, -0.1125, -0.1949, -0.228, 3.358, 0.05, 4.237, 0.1406, 10, 12.5, 0.5, 0.35, 0.0, 0.0, 0.0,
    0.0, 3.07, 1.85, 0.1125, -0.1949, -0.228, 3.358, 0.05, 4.237, 0.1406, 10, 12.5, 0.5, 0.35, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.114, 1.001, 0.59, 0.843, 10, 0.12, 0.04, 0.5, 0.5, 0.5, 1.5, 0.5, 0.005, 5.44, 0.443};

Real Stewart_q[] =
   {0.2, 0.3, -0.4, 0.1, -1.4, 0.1};
Real Stewart_qg[] =
   {0.25, 0.25, -0.45, 0.07, -1.7, 0.07};
Real Stewart_l[] =
   {3.0508, 3.2324, 3.2997, 3.4560, 3.5797, 3.6935};
Real Stewart_dq[] =
   {0.2, 0.3, -0.4, 0.1, -1.4, 0.1};
Real Stewart_ddq[]=
   {-10.0, -10.0, -10, -10.0, -10, -10};
Real Stewart_tddq[]=
   {0, 0, 0, 0, 0, 0};
Real Comm[]=
   {1, 0, 0, -10, 0, 0};
Real Tau[]=
   {126.219689, 789.968672, 0.708602, 79.122963, 81.806978, -31.61797};

int stewartmain(void)
{
   clock_t start, end;
   Matrix InitPlatt(7,17);
   Stewart platt_Stewart, platt2;
   Matrix _q(6,1), qg(6,1), _dq(6,1), _ddq(6,1), _l(6,1), To(6,1),
   V(6,1), L(6,1), tddq(6,1);
   int i;

   InitPlatt<<Stewart_Ini;
   _q << Stewart_q;
   qg << Stewart_qg;
   _l << Stewart_l;
   _dq << Stewart_dq;
   _ddq << Stewart_ddq;
   tddq << Stewart_tddq;
   V << Comm;


   platt_Stewart = Stewart(InitPlatt);
   // uncomment the next line to use the configuration file
   //platt_Stewart = Stewart("conf/stewart.conf","STEWART");
   platt_Stewart.set_q(_q);
   platt_Stewart.set_dq(_dq);
   platt_Stewart.set_ddq(_ddq);

   cout<<"===============================\n";
   cout<<"Benchmarks for Stewart platform\n";
   cout<<"===============================\n\n";
   cout<<"Inverse Kinematics :\n";
   start = clock();
   for(i =0; i<20000; i++)
      L = platt_Stewart.InvPosKine();
   end = clock();
   cout<<"Time : "<<((end - start)/(Real)CLOCKS_PER_SEC)*1000.0/20000<<"\nResult : "<<L.t()<<endl;


   cout<<"Forward Kinematics :\n";
   platt_Stewart.set_q(qg);
   start = clock();
   for(i =0; i<100; i++){
      L = platt_Stewart.ForwardKine(qg,_l);
      platt_Stewart.set_q(qg);}
   end = clock();
   cout<<"Time : "<<((end - start)/(Real)CLOCKS_PER_SEC)*1000.0/100<<"\nResult : "<<L.t()<<endl;

   platt_Stewart.set_q(_q);

   cout<<"Inverse Dynamics:\n";
   start = clock();
   for(i =0; i<100; i++)
      L = platt_Stewart.Torque();
   end = clock();
   cout<<"Time : "<<((end - start)/(Real)CLOCKS_PER_SEC)*1000.0/100<<"\nResult : "<<L.t()<<endl;

   platt_Stewart.set_ddq(tddq);
   To << Tau;
   cout<<"Forward Dynamics:\n";
   start = clock();
   for(i =0; i<100; i++)
      L = platt_Stewart.ForwardDyn(To);
   end = clock();
   cout<<"Time : "<<((end - start)/(Real)CLOCKS_PER_SEC)*1000.0/100<<"\nResult : "<<L.t()<<endl;


   double t = 0.01;

   L=platt_Stewart.ForwardDyn_AD(V,t);

   cout<<"Forward Dynamics with actuators:\nResult : "<<L.t()<<endl;

   return 0;
}

const Real PUMA560_data[] =
  {0, 0, 0, 0, M_PI/2.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.35, 0, 0, 0, 0, 0, 0, 0, 
   0, 0, 0, 0.4318, 0, 0, 0, 0, 17.4, -0.3638, 0.006, 0.2275, 0.13, 0, 0, 0.524, 0, 0.539, 0, 0, 0, 0, 0,
   0, 0, 0.15005, 0.0203, -M_PI/2.0, 0, 0, 0, 4.8, -0.0203, -0.0141, 0.07, 0.066, 0, 0, 0.086, 0, 0.0125, 0, 0, 0, 0, 0,
   0, 0, 0.4318, 0.0, M_PI/2.0, 0, 0, 0, 0.82, 0, 0.019, 0, 0.0018, 0, 0, 0.0013, 0, 0.0018, 0, 0, 0, 0, 0, 
   0, 0, 0, 0.0, -M_PI/2.0, 0, 0, 0, 0.34, 0.0, 0.0, 0.0, 0.0003, 0.0, 0.0, 0.0004, 0.0, 0.0003, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0, 0.09, 0.0, 0.0, 0.032, 0.00015, 0.0, 0.0, 0.00015, 0.0, 0.00004, 0, 0, 0, 0, 0};

int main(void)
{
   int i;
   clock_t start, end;
   Matrix initrob(6,23), thomo, temp2;
   ColumnVector q(6), qs(6), temp;
   Robot robot;

   initrob << PUMA560_data;
   robot = Robot(initrob);
   q = M_PI/6.0;

   q = M_PI/4.0;
   printf("=================================\n");
   printf("Benchmarks for serial 6 dof robot\n");
   printf("=================================\n\n");

   printf("Begin compute Forward Kinematics\n");
   start = clock();
   for (i = 1; i <= NTRY; i++) {
      robot.set_q(q);
      temp2 = robot.kine();
   }
   end = clock();
   printf("MilliSeconds %6.2f\n", ((end - start) / (Real)CLOCKS_PER_SEC)*1000.0/NTRY);
   printf("end \n");

   qs = 1.0;
   qs(1) = M_PI/16.0;
   robot.set_q(q);
   thomo = robot.kine();
   printf("Begin compute Inverse Kinematics\n");
   start = clock();
   for (i = 1; i <= NTRY; i++) {
      robot.set_q(qs);
      temp = robot.inv_kin(thomo);
   }
   end = clock();
   printf("MilliSeconds %6.2f\n", ((end - start) / (Real)CLOCKS_PER_SEC)*1000.0/NTRY);
   printf("end \n");

   printf("Begin compute Jacobian\n");
   start = clock();
   for (i = 1; i <= NTRY; i++) {
      robot.set_q(q);
      temp2 = robot.jacobian();
   }
   end = clock();
   printf("MilliSeconds %6.2f\n", ((end - start) / (Real)CLOCKS_PER_SEC)*1000.0/NTRY);
   printf("end \n");

   printf("Begin compute Torque\n");
   start = clock();
   for (i = 1; i <= NTRY; i++) {
      temp = robot.torque(q,q,q);
   }
   end = clock();
   printf("MilliSeconds %6.2f\n", ((end - start) / (Real)CLOCKS_PER_SEC)*1000.0/NTRY);
   printf("end \n");

   printf("Begin acceleration\n");
   start = clock();
   for (i = 1; i <= NTRY; i++) {
      temp = robot.acceleration(q,q,q);
   }
   end = clock();
   printf("MilliSeconds %6.2f\n", ((end - start) / (Real)CLOCKS_PER_SEC)*1000.0/NTRY);
   printf("end \n\n");

   stewartmain();
   return 0;
}



