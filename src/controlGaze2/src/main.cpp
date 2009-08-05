// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

/* 
 * Copyright (C) 2007 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author Manuel Lopes, Jonas Ruesch, Alexandre Bernardino
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

#include <iCub/control_gaze.h>

using namespace std;
using namespace yarp::os;

using namespace iCub::contrib;

/**
 * @ingroup icub_module
 *
 * \defgroup icub_controlGaze2	controlGaze2
 *
 * Head-Eye Controller Module. 
 *
 * Receives position / velocity comands either  
 * in angles (azimuth/elevation/vergence)[degrees] or
 * in image coordinates (horizontal, vertical, disparity)[check this]
 * and executes the comands by appropriately controlling 
 * the iCub head's joint angles.
 * 
 * Position commands are controlled in a saccadic manner: 
 * - fast relocation of eyes to the target direction, 
 * then the neck acompanies the motion while the eyes 
 * counter-rotate to keep the heading direction stable.
 * 
 * Velocity comands are executed in a smooth-pursuit mode:
 * - slower movements of eye/neck with eyes direction 
 * leading the head direction. 
 * 
 * Notifies other modules of important status data and events:
 * - start and end of saccade for visual saccadic suppression.
 * 
 * A mode for the integration of vestibular information is under development.
 *
 * A mode for the integration of motion prediction, either using integral control
 * of model based predictors, is under development.
 *
 *
 *
 * Parameters:
 * - appPath (string - defaults to "")
 *   Absolute path to the application folder.
 * - configCamera (string - defaults to "")
 *   Configuration file with camera parameters.
 * - imageSize (int,int)
 *   Actual image size (not calibration size)
 * - motorboard (string - defaults to "")
 *   Port name of the remote (server) control board.
 * - limitResetTime (double - defaults to 4)
 *   Time from start of saccade until looking back to center 
 *   if saccade reaches a joint limit.
 * - headSaccadeDelay (double - defaults to 0.0)
 *   Minimum time between saccades.
 * - FrameRate (double - defaults to 20 [Hz])
 *   Expected rate (samples per second) of input data in velocity mode (smooth pursuit)
 * - Control rate (double - defaults to 50 [Hz])
 *   Rate of the control thread. 
 *   Limited by the motor control board rates (50Hz is about the maximum).
 * - K (double[6]).
 *   Gains of the rate controller for head-eye coordination.
 * - egosphereVisualUpdateThreshold (double - defaults to 1)
 *   Threshold to check when a saccade reaches the end. 
 *   When the gaze error squared norm (azimuth_error^2+elevation_error^2) 
 *   is below this value, a message is sent to notify the end of the saccade.
 * - log (bool - defaults to false)
 *   If true, a log file is created with several important status variables.
 * - logfilename (string - defaults to "log")
 *   Name of the file where to log information.
 * - pidON (bool - defaults to false)
 *   EXPERIMENTAL. Turns on integral control.
 * - pidGAIN (double - defaults to 5)
 *   EXPERIMENTAL. Sets the gain for integral control.
 * - vorON (bool - defaults to false)
 *   EXPERIMENTAL. Turns on the vestibular (inertial) system.
 * - configPredictors (string - defaults to "")
 *   EXPERIMENTAL. Configuration file for model based predictors.
 * Commands:
 *
 * 	-- ACTUATION COMMANDS
 * - sac abs <x> <y> : saccade in absolute angles
 * - set pos <x> <y> : same as before
 * - sac rel <x> <y> : saccade in head relative angles
 * - sac img <x> <y> : saccade in camera normalized relative coordinates
 * - sac pix <x> <y> : saccade in image pixel relative coordinates
 * - pur abs <x> <y> : pursuit in absolute angles
 * - pur rel <x> <y> : pursuit in head relative angles
 * - pur img <x> <y> : pursuit in camera normalized relative coordinastes
 * - pur pix <x> <y> : pursuit in image pixel relative coordinates
 *  -- STATUS COMMANDS
 * - rset            : resets the state of the controller
 * - get st          : get time from last saccade
 * - get stat        : get controller status
 * - get ref         : get current gaze reference
 * - get dh          : get current head gaze
 * - get der         : get current right eye gaze
 * - get del         : get current left eye gaze
 *  -- CONFIGURATION COMMANDS
 * - set mst         : set minimum saccade time (a.k.a head saccade delay)
 * - set lrt         : ser joint limit reset time
 * - get mst         : get minimum saccade time (a.k.a head saccade delay)
 * - get lrt         : get joint limit reset time

 *   
 * \dot
 * digraph module_camcalib_example {
 *     graph [ rankdir = "LR" ];
 *     edge [arrowhead="open", style="solid"];
 *     node [shape=ellipse];
 *     subgraph cluster_camcalib {
 *      color = "black"; style = "solid";
 *      label = "control gaze module";
 *       "/camcalib/image";
 *     }
 *     "/attention" -> "/control_gaze/gazingtarget"
 *     "/control_gaze/gazingtarget" -> "/controlboard/"
 * \enddot
 *
 * \see iCub::contrib::Control_GazeModule
 *
 * \author Manuel Lopes
 *
 */


int main(int argc, char *argv[]) {

    Network yarp;
    Control_GazeModule module;
    module.setName("/controlGaze2"); // set default name of module
    return module.runModule(argc,argv);
}
