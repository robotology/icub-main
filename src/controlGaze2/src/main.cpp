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
 * Head-Eye Coordinated Control. 
 *
 * OBJECTIVES:
 *
 * ControlGaze2 is a module that controls in a coordinate way the head and the eyes of the iCub. 
 * It has been made for two main purposes:
 * - Specify the gaze direction of the robot in (azimuth,elevation,vergence) angles rather that raw head/eye joints.
 * - Provide a biological look to the dynamics of Head/Eye coordination.
 * - Provide a transparent and intuitive interface for controlling iCub's head, abtracting from low level control details. 
 * The module allows changing the gaze direction by:
 * - Specifying the absolute azimuth and elevation angles (in degrees).
 * - Specifying the azimuth and elevation angles relative to the current gaze direction.
 * - Specifying the normalized image coordinates where the robot should look at.
 * - Specifying the pixel coordinates in the image where we want the robot to look at. 
 *
 * OUTLOOK:
 *
 * The module controls simultaneously head gaze and vergence. Head gaze control has two modalities:
 * - Saccades
 * - Smooth Pursuit 
 * Saccades are controlled in two phases:
 * - Fast phase - the eyes are driven very quickly to the destination position
 * - Slow phase - the neck moves toward the final position in a slower velocity and the eyes counter-rotate to keep the image stable. 
 * A new saccade is accepted only when the previous one has finished. 
 * This is the typical operation in humans where saccades are used to change the object of interest.
 * Smooth pursuit only operates in the slow phase, but it accepts a continuous stream of commands. 
 * It is meant to emulate the human behavior when tracking an object.
 * A single set of gains suits both the saccade and smooth pursuit modes. 
 * These gains specify both the speed of the motions and the amount of motion that is performed by the eyes and by the neck.
 * Vergence also operates in a single phase. There is an independent gain for the vergence controller.
 *
 * OPERATION:
 * 
 * The module has an internal loop at 50Hz. 
 * At this rate it controls the angular velocities of each joint in order to reach a certain provided reference gaze value, in azimuth and elevation angles. 
 * This reference can be provided in 4 different coordinate systems.
 * - Absolute - specify the desired gaze direction as the absolute (body centered) azimuth and elevation angles.
 * - Relative - add azimuth and elevation angles to the current gaze direction.
 * - Pixel - this is a relative modality, but the relative angles are derived from the image pixel positions.
 * - Normalized - the same as before in normalized image coordinates [-1,1]. 
 * Usually there are no stability problems when controlling the module with saccade motions. 
 * The module does not accept new commands while the gaze has not reached the reference values. 
 * In pursuit mode, the heading direction can be specified at any time step and the controller must recompute the controls. 
 * This puts some stability issues because the trajectories become dependent of the rate commands are sent to the controller. 
 * The default gains have been tuned for a command refresh rate of 50 ms (20 Hz). 
 * 
 * CURRENT STATUS:
 * 
 * The following sections describe the current state of controlGaze2. 
 * A new module, controlGaze3 is under preparation to better cope with the new standards for application management, as well as for improvements in the interfaces.
 *
 * STARTING CONTROL GAZE 
 * 
 * To start controlGaze2 type execute the following command:
 * - controlGaze2 <parameters>
 * The following parameters are accepted:
 * 
 * \defgroup configCamera
 */
 /*@{*/
 /**
 * --configCamera iCubEyes.ini
 * 
 * A file with camera parameters. 
 * To convert from pixel coordinates to azimuth and elevation angles we need the intrinsic parameters of the cameras. 
 * These are the parameters that result from camCalibConf. The file should be something like this: 
 * ------------------------------------------------
 * [CAMERA_CALIBRATION_RIGHT]
 * projection pinhole
 * drawCenterCross 0
 * w  320
 * h  240
 * fx 224.34
 * fy 223.723
 * cx 178.58
 * cy 92.5934
 * k1 -0.381097
 * k2 0.153629
 * p1 0.0011246
 * p2 -0.00120395
 * [CAMERA_CALIBRATION_LEFT]
 * projection pinhole
 * drawCenterCross 0
 * w  320
 * h  240
 * fx 214.953
 * fy 213.455
 * cx 166.773 
 * cy 125.505
 * k1 -0.358372
 * k2 0.116527
 * p1 -0.000603539
 * p2 -0.000591796 
 * ----------------------------------------------------
 * If the images come from camCalib modules, the optical distortion parameters (k1, k2, p1, p2) should be put to 0. 
 /*@}*/
 /**
 * \defgroup appPath
 */
 /*@{*/
 /**
 * --appPath c:/icub/app/myapp
 * 
 * The folder name where to find the configuration files. This is subject to change in the new ResourceFinder specification. 
 /*@}*/
 /**
 * \defgroup imageSize
 */
 /*@{*/
 /**
 * --imageSize 160x120
 * 
 * If the image size is different from the specified in the calibration file, this allows rescaling the intrinsic parameters. 
 * If this is not specified, the module will use the image dimension specified in the calibration file.
 /*@}*/
 /**
 * \defgroup motorBoard
 */
 /*@{*/
 /**
 * --motorBoard /icub/head
 * 
 * This specifies the prefix of the port names made available by the control board to the head device. 
 * If you use ICubInterface, this is usually /icub/head. 
 /*@}*/
 /**
 * \defgroup Other Parameters and Defaults
 */
 /*@{*/
 /**
 * pidON	          0
 * pidGAIN	  5.0
 * vorON	          0
 * log	          0
 * K                0.04 0 -0.08 -0.024 0 -0.056
 * FrameRate	  20
 * ControlRate      50
 * limitResetTime   3.0
 * headSaccadeDelay 3.0
 * 
 * These could go to a initialization file (e.g. controlgaze2.ini)and invoked by:
 * - --file controlgaze2.ini
 * This is probably going to change in future version due to the new ResourceFinder's methods. 
 /*@}*/
 /**
 * TESTING THE MODULE FROM THE CONSOLE
 * 
 * The module has a console interface where you can issue the following commands.
 * If you have access to the console window, you can just type the commands there. Otherwise open a command window and type:
 * - yarp rpc /controlGaze2/conf
 * Then just type the required commands in the console.
 * 
 * ACTUATION COMMANDS
 *
 * - sac abs <azi> <elev> : saccade in absolute angles (degrees)
 * - set pos <azi> <elev> : same as before (obsolete but left here for historical reasons)
 * - sac rel <azi> <elev> : saccade in head angles (degrees) relative to the current position
 * - sac img <x> <y> : saccade in camera normalized relative coordinates (-1,1)
 * - sac pix <x> <y> : saccade in image pixel coordinates 
 * - pur abs <azispd> <elevspd> : pursuit in absolute angular velocities
 * - pur rel <azispd> <elevspd> : pursuit in angular velocities relative to the current ones (acelerates, decelerates)
 * - pur img <x> <y> : pursuit in camera normalized relative coordinates (-1,1)
 * - pur pix <x> <y> : pursuit in image pixel relative coordinates 
 * - TODO:
 * - verg abs <degrees>  : set the vergence angle 
 * - verg rel <degrees>  : set the vergence angle with respect to the current one.
 * - verg img <norm>     : set the vergence angle to compensate the existing disparity expressed in normalized coordinates.
 * - verg pix <disp>     : set the vergence angle to compensate the existing disparity expressed in pixels.
 *
 * STATUS COMMANDS
 *
 * - rset : resets the state of the controller
 * - get st : get time from last saccade
 * - get stat : get controller status
 * - get ref : get current gaze reference
 * - get dh : get current head gaze
 * - get der : get current right eye gaze
 * - get del : get current left eye gaze 
 * - TODO:
 * - get ver : get the current vergence angle (degrees)
 * - get rv   : get the reference vergence angle (degrees)
 * 
 * CONFIGURATION COMMANDS
 *
 * - set mst <float> : set minimum saccade time (a.k.a head saccade delay)
 * - set lrt <float> : ser joint limit reset time
 * - get mst         : get minimum saccade time (a.k.a head saccade delay)
 * - get lrt         : get joint limit reset time
 * - set verg <float>: sets the vergence controller gain
 * - get verg        : gets the vergence controller gain
 * 
 * For instance, if you issue the command:
 * - sac abs 10 10
 * The head-eye system will move to 10 degrees azimuth and 10 degrees elevation. 
 *
 * SENDING COMMANDS FROM OTHER MODULES 
 * 
 * You can issue the previous actuation commands from a module sending appropriate bottles to the /controlGaze2/conf port. 
 * For instance the following code will send a bottle instructing controlGaze2 to move to 10 degrees azimuth and 10 degrees elevation: 
 *
 * Bottle &bot = my_output_port.prepare();
 * bot.addVocab( Vocab::encode("sac") );
 * bot.addVocab( Vocab::encode("abs") );    
 * bot.addDouble(10.0);
 * bot.addDouble(10.0);
 * my_output_port.write();
 *
 * However, these commands use the RPC protocols and thus cannot be issued very fast (there is a delay due to the acknowledge/reply protocol). 
 * If you want to do send high frequency and low latency commands to the module, please use the streaming ports, described below. 
 * 
 * STREAMING PORTS
 * 
 * Beyond /controlGaze2/conf, that receives the console commands, the other ports opened by controlGaze2 are of the streaming type. 
 * They do not provide replies or acknowledges.
 * 
 * \defgroup /controlGaze2/pos
 */
 /*@{*/
 /**
 * This port accepts gaze position commands. 
 * The iCub head will perform a fast redirection of gaze (saccade), first moving the eyes quickly to the final heading. 
 * Then it will move the head, while the eyes counter-rotate to keep the image stabilized.
 * The module will ignore any position commands issued while performing the saccade. 
 * When the saccade finishes, other position commands can will be accepted.
 * To check if the module is accepting position commands, please see the documentation of port /status:o. States 3,4,5 and 6 will accept saccades.
 * 
 * Each command is a vector of 5 floats:
 * - horz_coord - the gaze value for the horizontal direction
 * - vert_coord - the gaze value for the vertical direction
 * - coord_sys - the type of coordinates/units related to the gaze values.
 * -#  'a' (cast to float)- the command is the absolute orientation (degrees).
 * -#   o 'r' (cast to float)- the command is the orientation relative to the current one (degrees).
 * -#   o 'p' (cast to float)- the command is given in normalized image coordinates (-1,1).
 * -#   o 'i' (cast to float)- the command is given in image pixel coordinates. 
 * type_behav (optional)- just use 's', if you what to specify the following argument.
 * saccade_id (optional) - a increasing number specifying the saccade order. In some cases saccade commands can be out of order. This is sometimes the case if you stream saccade commands. The attention system uses this. Most probably you will not need it. 
 *  
 /*@}*/
 /**
 * \defgroup /controlGaze2/vel
 */
 /*@{*/
 /**
 * This port accepts commands that implement the smooth pursuit behavior. 
 * The behavior is similar to the position commands but the eye does not saccade faster: both eyes and head move slower. 
 * The port accepts a fast stream of commands. Each command is a Vector of 3 floats:
 * - horz_coord- the gaze value for the horizontal direction
 * - vert_coord - the gaze value for the vertical direction
 * - coord_sys - the type of coordinates/units related to the gaze values.
 * -# 'a' (cast to float)- the command is the absolute orientation (degrees).
 * -# 'r' (cast to float)- the command is the orientation relative to the current one (degrees).
 * -# 'p' (cast to float)- the command is given in normalized image coordinates (-1,1).
 * -# 'i' (cast to float)- the command is given in image pixel coordinates. 
 *
/*@}*/
 /**
 * \defgroup /controlGaze2/dis
 */
 /*@{*/
 /**
 * This port accepts gaze vergence commands. Each command is a vector of 1 float:
 * - x - the horizontal image disparity in normalized image coordinates (-1, 1) 
 *
 /*@}*/
 /**
 * \defgroup /controlGaze2/imgcoord
 */
 /*@{*/
 /**
 * This port was made purposefully to connect directly viewer clicks. Will send the gaze direction to the clicked position in the viewer. 
 *
 /*@}*/
 /**
 * \defgroup /controlGaze2/imu
 */
 /*@{*/
 /**
 * Connects to the output of the inertial sensor a compensates for external body motions.
 /*@}*/
 /**
 * \defgroup /controlGaze2/trackersignal/bot:o 
 */
 /*@{*/
 /**
 * Not working. Used once to indicate the end of a saccade, meaning there is a new object to track. 
 * Please advice if interested in this feature. 
 *
 /*@}*/
 /**
 * \defgroup /controlGaze2/possibledirections/vec:o 
 */
 /*@{*/
 /**
 * Experimental. Tells external modules about direction that cannot be reached due to head-eye mechanical limits. 
 * Please advice if interested in this feature.
 * 
 /*@}*/
 /**
 * \defgroup /controlGaze2/status:o 
 */
 /*@{*/
 /**
 * Outputs the state of the controller and the current azimuth and elevation gaze direction coordinates (degrees).
 * The port sends bottles with three elements:
 * - status - an integer specifying the state of the controller:
 * -# 1 - starting a saccade (eye phase).
 * -# 2 - Continuing a saccade (neck phase).
 * -# 3 - Doing smooth pursuit.
 * -# 4 - Stuck in a limit.
 * -# 5 - Resting.
 * -# 6 - Starting smooth pursuit. 
 * - azimuth - a floating point number with the gaze direction azimuth (in degrees).
 * - elevation - a floating point number with the gaze direction azimuth (in degrees). 
 *
 *
 * SUMMARY:
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
 * \author Manuel Lopes, Alexandre Bernardino
 *
 */


int main(int argc, char *argv[]) {

	Network yarp;
	ResourceFinder rf;
	rf.setVerbose(true);
	rf.setDefaultConfigFile("controlGaze.ini"); //overridden by --from parameter
	rf.setDefaultContext("controlGaze"); //overridden by --context parameter
	rf.configure("ICUB_ROOT", argc, argv);
    Control_GazeModule module;
    return module.runModule(rf); 
}
