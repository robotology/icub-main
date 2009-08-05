// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2008 Micha Hersch, EPFL
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   micha.hersch@robotcub.org
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

/**
 * @ingroup icub_module
 *
 * \defgroup icub_body_schema_learning  body_schema_learning  
 * 
 * Learns the body schema from sensorimotor contingencies
 *
 *
 * \section intro_sec Description
 *
 * This module learns the body schema (that is something similar to the Denavit-Hartenberg
 * parameters) of a humanoid robot. This is performed online, as partly described in
 * the following paper: http://infoscience.epfl.ch/record/117918 .
 * There are two ways of updating the body schema. One is to track
 * a marker on placed on a limb (for example the hand). By knowing the position of the the marker
 * with respect to the eyes, the system can update the entire kinematic chain going from the eyes to
 * the marker. The second way is to analyze the visual flow produced by head movements and update
 * the kinematics of the head accordingly. 
 *
 * \section lib_sec Libraries
 * yarp_os
 *
 * \section parameters_sec Parameters
 *
 * Mandatory parameters
 *  - --structure <struct_file> A file describing the initial structure of the body schema. The format 
 * of the file is described in section \ref format_sec. It is the same file that is given as parameter of
 * the reaching module (cf ReachingModuleThread.h), the simulator module (cf KinematicSimulator.h), 
 * and the head_transfo module (cf HeadToBodyCenteredRef.h)
 *  - --arm_base <ab>: the name of the most proximal joint of the arm, as it appears in the file <struct_file>
 *  - --arm_end <ae>: the name of the most distal joint of the arm, as it appears in the file <struct_file>
 *  - --head_base <hb> the name of the most proximal joint of the arm, as it appears in the file <struct_file>
 *  - --head_end <he>: the name of the most distal joint of the arm, as it appears in the file <struct_file>
 *
 * Optional parameters
 *  - --stereo <stereoFoR>: the name of the joint to which the frame of reference of the stereovision system
 * is attached.
 *  - --marker <marked_joint>: the name of the joint to which a marker is attached. 
 *  - --eye <eye_joint>: the name of the joint to which the eye perceiving the optical flow is attached
 *  - --world <world_joint>: the name of the joint which is rigidly fixed to the world. 
 *  - --threshold <thresh> specifying the distance betweem the seen and predicted marker position above
 *  which the seen position is considered wrong (vision outlier). Default value is 500 [mm].
 *  - --frequency <freq>: The number of updates before the body schema is sent out. Default value is 50.  
 *  - --logfile <logfilename>: a file where to log all the inputs of the body schema update functions. This file must
 * exist prior to launching the module.
 *  - --catchup: if one wants to read the log file upon initialization. Used for continuing a previous learning
 * experiment
 *  - --simulation: if one wants to run in simulation mode, by just readng data from <logfilename>, without
 * opening any port
 *  - --random_head: if one wants to start with random head axes (actually, wrong but not random axes, uncomment
 * line in open function code to have them random)
 *
 * \section format_sec File format
 *
 *

 * \section portsa_sec Ports Accessed
 * \section portsc_sec Ports Created
 * Input ports
 * - <basename>/proprioception_head:i to get the joint position of the head (the chain going from <hb> to <he>).
 * Messages coming in this port are expected to have appropriate time stamps (to match the corresponding 
 * visual position). This port is typically connected to /icub/head/state:o
 * - <basename>/proprioception_arm:i to get the joint position of the arm (the chain going from <ab> to <ae>).
 * Messages coming in this port are expected to have appropriate time stamps (to match the corresponding 
 * visual position). This port is typically connected to /icub/right_arm/state:o
 * - <basename>/vision:i to get the seen position of the marker as '<markerX> <markerY> <markerZ>' Messages
 * coming in this port are expected to have appropriate time stamps (to match the corresponding joint angle 
 * position)
 * - <basename>/visual_rotation:i to get the observed rotation of the visual flow as '<rotX> <rotY> <rotZ>'
 * Messages coming in this port are expected to have appropriate time stamps (to match the corresponding
 * joint angle position) 
 * 
 *
 * Output ports:
 * - <basename>/cmd:i to get a command from the network. Currently, only the quit command is implemented
 * - <basename>/body_schema:o to send updated version of the body schema. The format is given by the 
 * KinematicTree:Serialize()/Deserialize() methods. 
 *
 *\section in_files_sec Input Data Files
 *  
 * - <struct_file>: see conf/icub_head_right_arm.xml for an example.
 * 
 * This files specifies the kinematic structure 
 * of the manipulator. The robot can be described by a "kinematic tree", having (for example)
 * the torso as root
 * where edges represent rigid connections (translation) and nodes represent joints (rotations).
 * (see the paper mentioned above for more details)
 * The file is an xml file with the following structure (see conf/icub_head_right_arm_tree.xml
 * for the file describing the kinematics of the icub right arm and head.)
 * \code
 * <Segment> r_sfe
 *  <Axis> 1 0 0  </Axis>
 *   <Range> -90 90 </Range>
 *   <Position> -100 0 0 </Position>      
 *   <Children>
 *     <Segment> r_saa
 *     ...
 *   </Children>
 *  </Segment>
 * \endcode
 *
 * In the example below r_sfe is the name of the proximal joint. The <Position> tag refers to
 * the position of the joint with resect to the previous joint, when in the zero joint position.
 * Similarly
 * the <Axis> tag refers the the rotation axis, when all previous joints are in the zero position. 
 * <Range> provides the joint angle boundaries in degrees, and <Children> announce the next link.
 * The tip of the manipulator is specified as an additional joint, but it has no <Axis> tag, only a
 * <Position> tag 

 * - <log_file> : See the description below if --catchup us given as argument this file is also an input
 * otherwise it is only an out file.
 *
 * \section out_data_sec Output Data Files
 *
 * - <log_file>
 * The log file has the following format: each line corresponds to an update of the body schema
 * lines corresponding the a marker based update:
 * \code
 * "1 <angle1> <angle2> .. <angleN> <markerX> <markerY> <markerZ> <threshold>"
 * \endcode
 * where <anglei> are the angles of the kinematic chain from <stereoFoR> to <marked_joint>, <markerX>,
 * <markerY>, <markerZ> is the observed postion of the marker, <threshold> is the <thresh> value used. 
 * 
 * lines corresponding the optical flow based update:
 * \code
 * "0 <angle1> <angle2> .. <angleN> <diff_angle1> <diff_angle2> .. <diff_angleN> <rotX> <rotY> <rotZ>" 
 * \endcode
 * where <anglei> are the angles of the kinematic chain from <eye> to <world> at time t0, <diff_anglei>
 * are small angular displacements of the same joints and <rotX> <rotY> <rotZ> describes the rotation of the
 * visual flow that this displacement caused. <rotX> <rotY> <rotZ> are the 3 independent quaternion components
 * describing the rotation (i.e the norm of the vector is sin(theta/2) where theta is the rotation angle.  
 *
 *\section conf_file_sec Configuration Files
 *
 * all the parameters can be written in a configuration file using the --file <config_file> argument 
 * This configuration file may look like this
 * \code
 * # initial body schema
 * structure conf/icub_head_right_arm_tree.xml
 * #name of joint attached to the eye
 * #(here we don't use eye_pan to keep stereovision accurate
 * eye eye_tilt
 * # name of joint attached to the world
 * world neck_tilt
 * # name of joint where the stereovision system is attached
 * stereo eye_tilt
 * # name of joint where the marker is attached
 * marker r_hand
 * # most proximal joint of the arm
 * arm_base r_sfe
 * # most distal joint of the arm
 * arm_end r_hand
 * # most proximal joint of the head
 * head_base neck_tilt
 * # most distal joint of the head 
 * head_end eyes
 * # size of buffer used to store proprioceptive input
 * bufferSize 200
 * # distance between perceived and predicted marker position above which
 * # perception is discarded as a vision outlier 
 * threshold 500
 * # file where to log the updates
 * logfile data/test8.txt
 * 
 * ## for just reading the log file
 * #simulation
 * 
 * ## for wrong head axis
 * #random_head
 * \endcode
 * \section tested_os_sec Tested OS
 * Developped and tested on Linux, partially on Windows. This module uses no weird library or commands, 
 * just standard c++ so it should be portable.
 *
 * \section example_sec Example Instantiation of the Module
 * \code
 * ./body_schema_module --file conf/config_body_schema.ini --name /body_schema
 * sleep 1
 * yarp connect icub/head/state:o /body_schema/proprioception_head
 * yarp connect icub/right_arm/state:o /body_schema/proprioception_arm
 * yarp connect stereovision/vision1:o /body_schema/vision:i
 * \endcode
 * where the stereovision/vision1:o is assumed to provide the 3d position of the
 * marker seen by the stereovision system (e.g. \ref stereoVisualTracker)
 * If you want to use the learning based on the optical flow you can add
 * \code
 * yarp connect /vFlow/visualRotation:o /body_schema/visual_rotation:i
 * \endcode
 * where  /vFlow/visualRotation:o outputs the optical flow (see \ref visual_rotation)
 * \author Micha Hersch 
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at src/lasaBodySchema/include/BodySchemaLearning.h 
 */


#ifndef __BODY_SCHEMA_LEARNING_H__
#define __BODY_SCHEMA_LEARNING_H__


#include "KinematicTree.h"
#include "BodySchema.h"
//#include "KChainOrientationBodySchema.h"
#include "yarp/os/all.h"
#include "SpecializedPort.h"
#include "yarp/os/Stamp.h"
#include <iostream>
#include <fstream>


#define WITH_KIN_TREE


class TimedBuffer;


class BodySchemaLearningModule: public Module{

 protected:

  // communication ports


    JointAngleDataPort proprioception_head;
    JointAngleDataPort proprioception_arm;
    CartesianDataPort vision;
    VisualRotationPort visual_rotation_port;
    CommandPort commands;

    BodySchemaDataPort bsPort;

#ifdef WITH_KIN_TREE
    KinematicTree *body;
    KinematicChain *eyes_arm;
    KinematicChain *eye_world;
    KinematicChain *head;
    KinematicChain *arm;
#else
    KChainBodySchema body;
#endif

    float outlier_thresh; 
  
      
    cart_vec_t markerPosition; /**< The position of the marker in an end-effector FoR*/
    cart_vec_t visualPosition; /**< Seen marker position in the head centered FoR */
    joint_vec_t arm_proprio;/**< arm configuration */  
    joint_vec_t head_proprio;/**< Head to world configuration */  

    float seenRotation[3];
    double t0,t1;

    int static_update;
    int rotation_update;
    
    
    int update_cnt; 
    int outputFrequency; /**< the number of updates before sending out the result */
    float *body_data;
    int body_data_size;  

    int arm_size;

    TimedBuffer *head_buffer;
    TimedBuffer *arm_buffer;
   
    // for logging 
    fstream log;
    bool logging;
    bool simul;

protected:
    int loadChain(KinematicChain *chain,Searchable &s,const char *from, 
                  const char *to, const char *from_def, const char *to_def);
    int listen();
    bool updateBody();
    void bufferProprioception(){};//to implement
    bool listenToStaticVision();
    bool listenToVisualRotation();
  //  sendOutput();

 public:
    BodySchemaLearningModule();
    virtual ~BodySchemaLearningModule(){};
    virtual bool open(Searchable& config);
    virtual bool close();
    virtual bool updateModule();
    bool simulUpdate();
};


class TimedBuffer{
protected:
    float *buffer;
    double *time_buf;
    int cnt;
    bool full;
    int width;
    int size;
public:
    TimedBuffer(int size,int width);
    ~TimedBuffer();
    void add(float *values, double time);
    int lookFor(double time);
    float *get(int index);
    double getTime(int index){return time_buf[index];}
};

#endif 
