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

2004/07/13: Ethan Tira-Thompson
    -Added support for newmat's use_namespace #define, using ROBOOP namespace

2005/11/06: Etienne Lachance
    - No need to provide a copy constructor and the assignment operator 
      (operator=) for Control_Select class. Instead we use the one provide by the
      compiler.
-------------------------------------------------------------------------------
*/


#ifndef CONTROL_SELECT_H
#define CONTROL_SELECT_H

/*!
  @file control_select.h
  @brief Header file for Control_Select class definitions.
*/

//! @brief RCS/CVS version.
static const char header_Control_Select_rcsid[] = "$Id: control_select.h,v 1.1 2007/07/24 16:03:09 amaldo Exp $";


#include <string>
#include "controller.h"

#ifdef use_namespace
namespace ROBOOP {
  using namespace NEWMAT;
#endif

#define NONE   0
#define PD     1
#define CTM    2
#define RRA    3
#define IMP    4

#define CONTROLLER  "CONTROLLER"

#define PROPORTIONAL_DERIVATIVE    "PROPORTIONAL_DERIVATIVE"
#define COMPUTED_TORQUE_METHOD     "COMPUTED_TORQUE_METHOD"
#define RESOLVED_RATE_ACCELERATION "RESOLVED_RATE_ACCELERATION"
#define IMPEDANCE                  "IMPEDANCE"

/*!
  @class Control_Select
  @brief Select controller class.

  This class contains an instance of each controller class. The active controller
  will be selected when reading a controller file. "type" value correspond to the
  active controller, ex:
  \li <tt> type = NONE </tt>: no controller selected
  \li <tt> type = PD </tt>: Proportional Derivative 
  \li <tt> type = CTM </tt>: Computer Torque Method
  \li <tt> type = RRA </tt>: Resolved Rate Acceleration
  \li <tt> type = IMP </tt>: Impedance

  Bellow is an exemple of RRA configuration file (more info on configuration
  file in config.h/cpp):

\verbatim
  [CONTROLLER]

  type:   RESOLVED_RATE_ACCELERATION
  dof:    6

  [GAINS]

  Kvp:         500.0
  Kpp:        5000.0
  Kvo:         500.0
  Kpo:        5000.0
\endverbatim
*/
class Control_Select
{
public:
    Control_Select();
    Control_Select(const std::string & filename);
    int get_dof();
    void set_control(const std::string & filename);
    Proportional_Derivative pd;
    Computed_torque_method ctm;
    Resolved_acc rra;
    Impedance impedance;

    short type,            //!< Type of controller: PD, CTM,...
	  space_type;      //!< JOINT_SPACE or CARTESIAN_SPACE.
    std::string ControllerName; //!< Controller name.
private:
    int dof;               //!< Degree of freedom.
};

#ifdef use_namespace
}
#endif

#endif















