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

2006/05/19: Richard Gourdeau
    -Fix set_q, setq_p bug in xdot (reported by Philip Gruebele)
-------------------------------------------------------------------------------
*/

/*!
  @file dynamics_sim.cpp
  @brief Basic dynamics simulation class.
*/

//! @brief RCS/CVS version.
static const char rcsid[] = "$Id: dynamics_sim.cpp,v 1.1 2007/07/24 16:03:09 amaldo Exp $";

#include "dynamics_sim.h"
#include "robot.h"

#ifdef use_namespace
namespace ROBOOP {
  using namespace NEWMAT;
#endif

Dynamics *Dynamics::instance = 0;


/*!
  @fn Dynamics::Dynamics(Robot_basic *robot)
  @brief Constructor
*/
Dynamics::Dynamics(Robot_basic *robot_): robot(robot_)
{
  ndof = 1;
  dof_fix = 1;
  if(robot)
    {
      ndof = robot->get_dof();
      dof_fix = ndof + robot->get_fix();
    }

  q = ColumnVector(ndof); 
  q = 0;
  qp = ColumnVector(ndof);
  qp = 0;
  qpp = ColumnVector(ndof);
  qpp = 0;
  qd = ColumnVector(ndof);
  qd = 0.0;
  qpd = ColumnVector(ndof);
  qpd = 0;
  qppd = ColumnVector(ndof);
  qppd = 0;
  tau = ColumnVector(ndof);
  tau = 0.0;
  pd = ColumnVector(3); pd = 0;
  ppd = ColumnVector(3);
  ppd = 0;
  pppd = ColumnVector(3);
  pppd = 0;
  wd = ColumnVector(3);
  wd = 0;
  wpd = ColumnVector(3);
  wpd = 0;
  nsteps = 50;
  to = 0;
  tf = 0.1;
  dt = ((tf-to)/nsteps)/4.0;
  tf_cont = 1;

  first_pass_Kutta = true;
  instance = this;
}

Dynamics *Dynamics::Instance()
/*!
  @brief A pointer to Dynamics instance. Pointer is 0 if there 
         is no instance (logic done in Constructor).
*/
{
    return instance;
}

void Dynamics::set_dof(Robot_basic *robot_)
/*!
  @brief Set the degree of freedom.

  Obtain the degree of freedom from Robot_basic pointer. Some vectors
  will be resize with new current dof value.
*/
{
  ndof = 1;
  dof_fix = 1;
    robot = robot_;
  if(robot)
    {
      ndof = robot->get_dof();
      dof_fix = ndof + robot->get_fix();
    }
  q = ColumnVector(ndof); 
  q = 0;
  qp = ColumnVector(ndof);
  qp = 0;
  qpp = ColumnVector(ndof);
  qpp = 0;
  qd = ColumnVector(ndof);
  qd = 0.0;
  qpd = ColumnVector(ndof);
  qpd = 0;
  qppd = ColumnVector(ndof);
  qppd = 0;
  tau = ColumnVector(ndof);
  tau = 0.0;
  
  first_pass_Kutta = true;
}

void Dynamics::set_time_frame(const int nsteps_)
//! @brief Set the number of iterations.
{ 
    nsteps = nsteps_; 
}

void Dynamics::set_final_time(const double tf)
//! @brief Set the file time.
{ 
    tf_cont = tf; 
}

void Dynamics::reset_time()
//! @brief Set the simulation time to the inital time.
{
    first_pass_Kutta = true;
}

short Dynamics::set_controller(const Control_Select & control_)
//! @brief Set the control variable from the Control_Select reference.
{
    control = control_;
    if( (ndof != control.get_dof()) && (control.type != NONE) )
      {
	control.type = NONE;
	cerr << "Dynamics::set_controller: mismatch degree of freedom between robot and controller." << endl;
	return -1;
      }
    return 0;
}

short Dynamics::set_trajectory(const Trajectory_Select & path_select_)
//! @brief Set the path_select variable from the Trajectory_Select reference.
{
    path_select = path_select_;

    if (control.space_type != path_select.type)
      {
	control.type = NONE;
	path_select.type = NONE;
	cerr << "Dynamics::set_trajectory: controller space and trajectory space are not compatible.\n" 
	     << "                          Both should be in joint space or in cartesian space." << endl;
	return -1;
      }
    return 0;
}

ReturnMatrix Dynamics::set_robot_on_first_point_of_splines()
/*!
  @brief Set the robot on first point of trajectory.

  Assigned the robot joints position to the first point of the trajectory 
  if the latter is expressed in joint space, or assigned the robot joints
  position via inverse kinematics if the trajectory is expressed in 
  cartesian space.
  The function return a message on the console if the format of the 
  trajectory file is incorrect.
*/
{
  ColumnVector qs(2*ndof);
  
  if(path_select.type == JOINT_SPACE)
    {
      if(path_select.path.p_pdot(0.0, qd, qpd))
	cerr << "Dynamics::set_robot_on_first_point_of_spines: invalid joint space path." << endl;
      else
	{
	  tf_cont = path_select.path.get_final_time();
	  robot->set_q(qd);
	  qs.SubMatrix(1,ndof,1,1) = qd;
	  qs.SubMatrix(ndof+1,2*ndof,1,1) = 0;
	  qs.Release();
	  return qs;
	}
    }
  else if(path_select.type == CARTESIAN_SPACE) // && quaternion active
    {
      if( (path_select.path.p_pdot_pddot(0.0, pd, ppd, pppd) == 0) && 
	  (path_select.path_quat.quat_w(0.0, quatd, wd) == 0) )
	{
	  Matrix Tobj(4,4); Tobj = 0;
	  Tobj.SubMatrix(1,3,1,3) = quatd.R();
	  Tobj.SubMatrix(1,3,4,4) = pd;
	  Tobj(4,4) = 1;
	  bool converge;
	  robot->inv_kin(Tobj, 0, converge);
	  
	  if(converge)
	    {
	      tf_cont = path_select.path.get_final_time();
	      q = robot->get_q();
	      qs.SubMatrix(1,ndof,1,1) = q;
	      qs.SubMatrix(ndof+1,2*ndof,1,1) = 0;
	      
	      qs.Release();
	      return qs;
	    }
	}
      else
	cerr << "Dynamics::set_robot_on_first_point_of_spines: invalid cartesian path." << endl;
    }
  
  q = robot->get_q();
  qs.SubMatrix(1,ndof,1,1) = q;
  qs.SubMatrix(ndof+1,2*ndof,1,1) = 0;
  qs.Release();
  return qs;
}

ReturnMatrix Dynamics::xdot(const Matrix & x)
/*!
  @brief Obtain state derivative.
  @param x: state vector (joint speed and joint velocity).

  The controller torque is applied if any controller has been
  selected, then the joint acceleration is obtained.
*/
{
   q = x.SubMatrix(1,ndof,1,1);        // joint position from state vecteur
   qp = x.SubMatrix(ndof+1,2*ndof,1,1);// " "   velocity    "           "
   robot->set_q(q);
   robot->set_qp(qp);

   if(robot)
   {
       switch (control.type)
       {
	   case PD:
	       if(path_select.type == JOINT_SPACE)
	       {
		   if(path_select.path.p_pdot(time, qd, qpd))
		   {
		       xd = qp & qpp;   		   
		       xd.Release(); 
		       return xd;
		   }
	       }
	       else if(path_select.type == CARTESIAN_SPACE)
		   cerr << "Dynamics::xdot: Cartesian path can not be used with CTM controller." << endl;

	       tau = control.pd.torque_cmd(*robot, qd, qpd);

	       break;

	   case CTM:
	       if(path_select.type == JOINT_SPACE)
	       {
		   if(path_select.path.p_pdot(time, qd, qpd))
		   {
		       xd = qp & qpp;   		   
		       xd.Release(); 
		       return xd;
		   }
	       }
	       else if(path_select.type == CARTESIAN_SPACE)
		   cerr << "Dynamics::xdot: Cartesian path can not be used with CTM controller." << endl;

	       tau = control.ctm.torque_cmd(*robot, qd, qpd, qppd);

	       break;

	   case RRA:
	       if(path_select.type == CARTESIAN_SPACE)
	       {
		   if (path_select.path.p_pdot_pddot(time, pd, ppd, pppd) ||
		       path_select.path_quat.quat_w(time, quatd, wd)) 
		   {
		       xd = qp & qpp;
		       xd.Release();
		       return xd;
		   }
	       }
	       else if(path_select.type == JOINT_SPACE)
		   cerr << "Dynamics::xdot: Joint Space path can not be used with RRA controller." << endl;

	       tau = control.rra.torque_cmd(*robot, pppd, ppd, pd, wpd, wd, quatd, dof_fix, dt);
	       break;
	   default:
	       tau = 0;
       }
       qpp = robot->acceleration(q, qp, tau);
   }

   plot();

   xd = qp & qpp;  // state derivative

   xd.Release(); 
   return xd;
}

void Dynamics::Runge_Kutta4_Real_time()
//! @brief Runge Kutta solver for real time.
{
    if(first_pass_Kutta)
    {
	h = (tf-to)/nsteps;
	h2 = h/2.0;
	time = to;
	x = set_robot_on_first_point_of_splines();
	first_pass_Kutta = false;
	return;
    }

    k1 = xdot(x) * h;
    time += h2;
    k2 = xdot(x+k1*0.5)*h;
    k3 = xdot(x+k2*0.5)*h;
    time += h2;
    k4 = xdot(x+k3)*h;
    x = x + (k1*0.5+ k2 + k3 + k4*0.5)*(1.0/3.0);
}

void Dynamics::Runge_Kutta4()
//! @brief Runge Kutta solver.
{
   x = set_robot_on_first_point_of_splines();
   h = (tf_cont - to)/nsteps;
   h2 = h/2.0;
   time = to;

   for (int i = 1; i <= nsteps; i++) {
           k1 = xdot(x) * h;
      k2 = xdot(x+k1*0.5)*h;
      k3 = xdot(x+k2*0.5)*h;
      k4 = xdot(x+k3)*h;
      x = x + (k1*0.5+ k2 + k3 + k4*0.5)*(1.0/3.0);
      time += h;
   }
}

#ifdef use_namespace
}
#endif











