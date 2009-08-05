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

Reference:

[1] J. Angeles, Fundamentals of Robotic Mechanical Systems: Theory, Methods and Algorithms,
    Springer-Verlag, Mechanical Engineering Series, 1997, ISBN: 0-387-94540-7.

[2] E.B. Dam, M. Koch and M. Lillholm, "Quaternions, Interpolation and Animation",
    tech report of University of Copenhagen, number: DIKU-TR-98/5, 1998.

-------------------------------------------------------------------------------
Revision_history:

2004/05/22: Etienne Lachance
   -Added class Trajectory_Select.

2004/07/01: Etienne Lachance
   -Added doxygen documentation.

2004/07/01: Ethan Tira-Thompson
    -Added support for newmat's use_namespace #define, using ROBOOP namespace

2005/11/06: Etienne Lachance
    - No need to provide a copy constructor and the assignment operator 
      (operator=) for Spl_Quaternion, Spl_Cubic and Spl_path classes. Instead we 
      use the one provide by the compiler.
-------------------------------------------------------------------------------
*/


#ifndef TRAJECTORY_H
#define TRAJECTORY_H

/*!
  @file trajectory.h
  @brief Header file for trajectory generation class.
*/

//! @brief RCS/CVS version.
static const char header_trajectory_rcsid[] = "$Id: trajectory.h,v 1.1 2007/07/24 16:03:10 amaldo Exp $";


#ifdef _MSC_VER                         // Microsoft
//#include <string.h>
//#include <iostream.h>
#pragma warning (disable:4786)  /* Disable decorated name truncation warnings */
#endif
//#include <string>
//#include <iostream>

//#include <fstream>
#include <sstream>
#include <map>
#include "quaternion.h"
#include "utils.h"

#ifdef use_namespace
namespace ROBOOP {
  using namespace NEWMAT;
#endif

#define K_ZER0          1
#define BAD_DATA       -1
#define EXTRAPOLLATION -2
#define NOT_IN_RANGE   -3

/*!
  @class Spl_cubic
  @brief Natural cubic splines class.
*/
class Spl_cubic
{
public:
   Spl_cubic(){};
   Spl_cubic(const Matrix & pts);
   short interpolating(const Real t, ColumnVector & s);
   short first_derivative(const Real t, ColumnVector & ds);
   short second_derivative(const Real t, ColumnVector & dds);
private:
   int nb_path;    //!< Number of path, i.e: path in x,y,z nb_path=3
   Matrix
   Ak, Bk, Ck, Dk;
   RowVector tk;   //!< Time at control points.
   bool bad_data;  //!< Status flag.
};


#define NONE            0
#define JOINT_SPACE     1
#define CARTESIAN_SPACE 2

//! @brief Data at control points.
typedef std::map< Real, ColumnVector, less< Real > > point_map;


/*!
  @class Spl_path
  @brief Cartesian or joint space trajectory
*/
class Spl_path : public Spl_cubic
{
public:
   Spl_path():Spl_cubic(){};
   Spl_path(const std::string & filename);
   Spl_path(const Matrix & x);
   short p(const Real time, ColumnVector & p);
   short p_pdot(const Real time, ColumnVector & p, ColumnVector & pdot);
   short p_pdot_pddot(const Real time, ColumnVector & p, ColumnVector & pdot,
                      ColumnVector & pdotdot);
   short get_type(){ return type; }
   double get_final_time(){ return final_time; }
   
private:
   short type;        //!< Cartesian space or joint space.
   double final_time; //!< Spline final time.
};


//! @brief Data at control points.
typedef std::map< Real, Quaternion, less< Real > > quat_map;


/*!
  @class Spl_Quaternion
  @brief Cubic quaternions spline.
*/
class Spl_Quaternion
{
public:
   Spl_Quaternion(){}
   Spl_Quaternion(const std::string & filename);
   Spl_Quaternion(const quat_map & quat);
   short quat(const Real t, Quaternion & s);
   short quat_w(const Real t, Quaternion & s, ColumnVector & w);
private:
   quat_map quat_data;  //!< Data at control points.
};


/*!
  @class Trajectory_Select
  @brief Trajectory class selection.
*/
class Trajectory_Select
{
public:
    Trajectory_Select();
    Trajectory_Select(const std::string & filename);
    Trajectory_Select & operator=(const Trajectory_Select & x);

    void set_trajectory(const std::string & filename);

    short type;               //!< Cartesian or joint space
    Spl_path path;            //!< Spl_path instance.
    Spl_Quaternion path_quat; //!< Spl_Quaternion instance.
private:
    bool quaternion_active;   //!< Using Spl_Quaternion.
};

#ifdef use_namespace
}
#endif

#endif
