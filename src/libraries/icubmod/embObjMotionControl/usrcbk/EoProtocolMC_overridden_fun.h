/*
 * Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
 * Author:  Marco Accame
 * email:   marco.accame@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOPROTOCOLMC_OVERRIDDEN_FUN_H_
#define _EOPROTOCOLMC_OVERRIDDEN_FUN_H_

// this file is to be used by the PC104, but not anymore

#if 1

	#define OVERRIDE_eoprot_fun_UPDT_mc_joint_config

	#define OVERRIDE_eoprot_fun_UPDT_mc_joint_config_pidposition
	#define OVERRIDE_eoprot_fun_UPDT_mc_joint_config_impedance
	#define OVERRIDE_eoprot_fun_UPDT_mc_joint_config_limitsofjoint
	//#define OVERRIDE_eoprot_fun_UPDT_mc_joint_config_controlmode
	#define OVERRIDE_eoprot_fun_UPDT_mc_joint_config_pidtorque

	#define OVERRIDE_eoprot_fun_UPDT_mc_joint_status
	#define OVERRIDE_eoprot_fun_UPDT_mc_joint_status_basic
	#define OVERRIDE_eoprot_fun_UPDT_mc_joint_status
  #define OVERRIDE_eoprot_fun_UPDT_mc_joint_status_interactionmodestatus

	#define OVERRIDE_eoprot_fun_UPDT_mc_joint_cmmnds_setpoint

	#define OVERRIDE_eoprot_fun_UPDT_mc_motor_config
	#define OVERRIDE_eoprot_fun_UPDT_mc_motor_config_maxcurrentofmotor

	#define OVERRIDE_eoprot_fun_UPDT_mc_motor_status_basic

#endif




#endif  // include-guard

// eof

