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

-------------------------------------------------------------------------------
Revision_history:

2005/06/10: Etienne Lachance
    -The desired joint acceleration was missing in the computed torque method.
-------------------------------------------------------------------------------
*/


#ifndef DYNAMICS_SIM_H
#define DYNAMICS_SIM_H

/*!
  @file dynamics_sim.h
  @brief Header file for Dynamics definitions.
*/

//! @brief RCS/CVS version.
static const char header_dynamics_sim_rcsid[] = "$Id: dynamics_sim.h,v 1.1 2007/07/24 16:03:09 amaldo Exp $";

#include "control_select.h"
#include "quaternion.h"
#include "trajectory.h"
#include "utils.h"

#ifdef use_namespace
namespace ROBOOP {
  using namespace NEWMAT;
#endif

class Robot_basic;


/*!
  @class Dynamics
  @brief Dynamics simulation handling class.
*/
class Dynamics
{
public:
    Dynamics(Robot_basic *robot_);
    static Dynamics *Instance();
    virtual ~Dynamics(){}
    void set_dof(Robot_basic *robot_);
    short set_controller(const Control_Select & x);
    short set_trajectory(const Trajectory_Select & x);
    ReturnMatrix set_robot_on_first_point_of_splines();
    void set_time_frame(const int nsteps);
    void set_final_time(const double tf);
    void reset_time();
    void Runge_Kutta4_Real_time();
    void Runge_Kutta4();

    virtual void plot(){} //!< Virtual plot functions.

// private:
    ReturnMatrix xdot(const Matrix & xin);

    bool first_pass_Kutta; //!< First time in all Runge_Kutta4 functions.
    int ndof,              //!< Degree of freedom.
        dof_fix,           //!< Degree of freedom + virtual link.
        nsteps;            //!< Numbers of iterations between.
    double h,              //!< Runge Kutta temporary variable.
           h2,             //!< Runge Kutta temporary variable.
           time,           //!< Time during simulation.
           to,             //!< Initial simulation time.
           tf,             //!< Final time used in Runge_Kutta4_Real_time.
           tf_cont,        //!< Final time used in Runge_Kutta4.
           dt;             //!< Time frame.
    Matrix k1,             //!< Runge Kutta temporary variable.
           k2,             //!< Runge Kutta temporary variable.
           k3,             //!< Runge Kutta temporary variable.
           k4,             //!< Runge Kutta temporary variable.
           x,              //!< Stated vector obtain in Runge Kutta functions.
           xd;             //!< Statd vector derivative obtaint in xdot function.
    ColumnVector q,        //!< Joints positions.
                 qp,       //!< Joints velocities.
                 qpp,      //!< Joints accelerations.
                 qd,       //!< Desired joints positions.
                 qpd,      //!< Desired joints velocities.
                 qppd,     //!< Desired joints accelerations.
                 tau,      //!< Controller output torque.
                 pd,       //!< Desired end effector cartesian position.
                 ppd,      //!< Desired end effector cartesian velocity.
                 pppd,     //!< Desired end effector cartesian acceleration.
                 wd,       //!< Desired end effector cartesian angular velocity.
                 wpd;      //!< Desired end effector cartesian angular acceleration.
    Quaternion quatd;      //!< Desired orientation express by a quaternion.
    Control_Select control;        //!< Instance of Control_Select class.
    Trajectory_Select path_select; //!< Instance of Trajectory_Select class.
    Robot_basic *robot;            //!< Pointer on Robot_basic class.

    static Dynamics *instance;     //!< Static pointer on Dynamics class.
};

#ifdef use_namespace
}
#endif

#endif

