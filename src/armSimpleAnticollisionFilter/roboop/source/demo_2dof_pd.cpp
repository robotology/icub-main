/*
ROBOOP -- A robotics object oriented package in C++
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

-------------------------------------------------------------------------------
Revision_history:

2004/07/01: Etienne Lachance
    -Added support for newmat's use_namespace #define, using ROBOOP namespace
*/
/*! 
  @file demo_2dof_pd.cpp
  @brief A demo file.

  This demo file shows a two degree of freedom robots controller by
  a pd controller. The robot is define by the file "conf/rr_dh.conf", 
  while the controller is defined by the file "conf/pd_2dof.conf". The
  desired joint trajectory is defined by the file "conf/q_2dof.dat";
*/

//! @brief RCS/CVS version.
static const char rcsid[] = "$Id: demo_2dof_pd.cpp,v 1.1 2007/07/24 16:03:09 amaldo Exp $";

#include "gnugraph.h"
#include "controller.h"
#include "control_select.h"
#include "dynamics_sim.h"
#include "robot.h"
#include "trajectory.h"

#ifdef use_namespace
using namespace ROBOOP;
#endif

/*!
  @class New_dynamics
  @brief This is an example of customize Dynamics class.

  This class enherite from Dynamics class. At every time
  frame the new virtual plot functions record the current time
  and the robot joints positions. The data can then be
  used to create a plot.
*/
class New_dynamics: public Dynamics
{
public:
  New_dynamics(Robot_basic *robot_);
  virtual void plot();

  Robot_basic *robot;    /**< Robot_basic pointer.         */
  bool first_pass_plot;  /**< First time in plot function. */
  RowVector tout;        /**< Output time vector.          */
  Matrix xout;           /**< Output state vector.         */
  int i;                 /**< Temporary index.             */
};

/*! 
  @fn New_dynamics::New_dynamics(Robot_basic *robot)
  @brief Constructor
*/
New_dynamics::New_dynamics(Robot_basic *robot_): Dynamics(robot_)
{
  robot = robot_;
  first_pass_plot = true;
  i = 1;
}

/*! 
  @fn void New_dynamics::plot()
  @brief Customize plot function.

  Record the time (tout) and the joints positions (xout). This
  member function is call by the member function xdot.
*/
void New_dynamics::plot()
{
  if(first_pass_plot)
    {
      xout = Matrix(x.Nrows(),(int)(nsteps*(tf_cont-to)+1)*x.Ncols());
      xout.SubMatrix(1,x.Nrows(),1,x.Ncols()) = x;
      tout = RowVector((int)(nsteps*(tf_cont-to)+1));
      tout(1) = to;
      i = 0;
      first_pass_plot = false;
    }

  if(robot)
    {
      tout(i+1) = time;
      xout.SubMatrix(1,x.Nrows(),i*x.Ncols()+1,(i+1)*x.Ncols()) = x;
      i++;
    }
}


int main()
{
  
  Robot robot("conf/rr_dh.conf", "rr_dh");

  Trajectory_Select path("conf/q_2dof.dat");
  Control_Select control("conf/pd_2dof.conf");

  New_dynamics dynamics(&robot);

  dynamics.set_controller(control);
  dynamics.set_trajectory(path);
  dynamics.set_time_frame(500);

  dynamics.Runge_Kutta4();

  set_plot2d("Robot joints position", "time (sec)", "q(i) (rad)", "q", DATAPOINTS,
	     dynamics.tout, dynamics.xout, 1, 2);
 
   return(0);
}


