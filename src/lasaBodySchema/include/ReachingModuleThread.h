// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2008 
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
 * @ingroup icub_lasabodyschema_module
 * \defgroup icub_reaching_module  reaching_module  
 * 
 * Performs reaching movements with any serial manipulator
 *
 *
 * \section intro_sec Description
 *
 * This module performs reaching movements with a serial manipulator of arbitrary 
 * geometry and number of degrees of freedom. The target of the reaching movement 
 * can be a given position or a given position and orientation. The algorithm used
 * is the dynamical systems - based, hybrid joint angle and end-effector location
 * controller, described in this paper: http://infoscience.epfl.ch/record/114045
 * This module wasa used to perform reaching with the icub right arm and head tracking
 * with the icub head.
 *
 * \section lib_sec Libraries
 * yarp_os
 *
 * \section parameters_sec Parameters
 *
 * - --structure <struct_file> : an xml file describing the kinematic structure of the manipulator.
 * This parameter is mandatory. The format of this file is described in section \ref in_file_sec
 * - --from <prox_joint> the most proximal joint controlled
 * - --to <dist_joint> the most distal joint controlled. Only the position of this joint is taken into a
 * account, the joint itself is not controlled.  
 *
 * Optional parameters
 * - --name <basename> : base name of all created ports.
 * - --granularity <gran>:  the integration constant of the dynamical system in [ms], default value=10
 * - --orientation 1 :  if you want to specify the target orientation as well as position
 * - --pointing: if pointing is allowed (the robot points if it cannot reach)
 * - --jointControl: if you want a pure angular controller
 * - --simulation if you want to use just a simulator, not the real robot
 *
 *
 * The following parameters are needed when controling the actual robot (and not a simulator).
 *
 *   --part <part_name>: name of the part you ar controlling (default is right_arm). For now,
 *     the module cannot control more than one part
 * - --nbDofs <nb> : the number of DoFs sent as output.
 * - --controlGains <g1> <g2> ... <g<nb>>: the gain sent to the velocity control for each motor
 * - --maxVelocity <mv1> <mv2> ... <mv<nb>>: the maximum velocity for each motor 
 * 
 * All those parameters can be specified through a configuration file <config>
 * - --file <config> : name of configuration file where to read the other parameters 
 *
 *
 * \section portsa_sec Ports Accessed
 *
 * When actually controlling the robot, this accesses the velocityControl ports:
 * - /icub/vc/right_arm/fastCommand
 * - /icub/vc/right_arm/input
 * 
 *In order to do so, one must #define ICUB_IS_HERE 
 * 
 * \section portsc_sec Ports Created
 * input ports:
 * - <basename>/target:i to get the reaching target. Format is either 'x y z' if only the position is
 * given or 'x y z qx qy qz' if position and rotation are specified (--orientation 1 option). 
 * There qx qy qz are the rodrigues vector of the desired rotation, that is the complex part
 * of the corresponding quaternion, or in other word sin(theta/2)*a, where a is the rotation axis.
 * The x,y,z axes are specified by the kinematic structure file <struct_file> 
 * If there is no input connection to this port, random target are generated (un\#define RANDOM_TARGET to 
 * remove this option). The target is given in the chain base frame of reference (usually the torso). 
 * You can use the \ref kinematic_simulator to visualize if the target is you are giving the module is
 * in the right frame of reference. 
 *
 * - <basename>/cmd:i to get command from the network. Currently, only the quit command is implemented
 * - <basename>/body_schema:i to receive updated version of the body schema
 * - <basename>/robotState:i to receive the actual state of the robot, to match the initial position.
 *
 * output ports:
 * - <basename>/vc_command:o sends the joint angle commands (in degrees) sent to the robot
 * this port is supposed to be connected to the velocityControl module. The format is the one defined by
 * velocityControl, i.e "1 <angle> 2 <angle> ...", wher the numbers refer to the no of the joint in the part
 * and <angle> is the angle command in degrees. 
 * 
* \section in_file_sec Input Data File
 *
 * The file <struct_file> specifes the kinematic structure 
 * of the manipulator. Typically, this file describes the kinematic structure of the entire robot, but the
 * module can only control a particular kinematic chain withing this robot (for example the arm or the head).
 * The chain that is controlled for the reaching is specified by the --from and --to parameters.
 * The robot can be described by a "kinematic tree", having (for example)
 * the torso as root
 * where edges represent rigid connections (translation) and nodes represent joints (rotations).
 * (see my IJHR paper for more details)
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
 *
 *\section out_data_sec Output Data Files
 * none
 *
 * \section conf_file_sec Configuration Files
 * All the parameters can be written in a file given by the --file option on the command line. Here is
 * an example for reaching with the right arm
 * \code
 * structure ./conf/icub_head_right_arm_tree_safe.xml 
 * from r_sfe
 * to r_hand
 * part right_arm
 * nbDOFs 7
 * controlGains 3 3 3 3 3 3 3
 * maxVelocity 10 10 10 10 10 10 10
 * \endcode
 * 
 * Here is another example for reaching with the gaze, i.e. tracking the the head and eyes
 * \code
 * structure ./conf/icub_head_right_arm_tree_safe.xml 
 * from neck_tilt
 * to gaze
 * part head
 * pointing
 * jointControl
 * part head
 * nbDOFs 4
 * controlGains 2 1 1 1
 * maxVelocity 15 15 15 15
 * \endcode
 *
 *\section tested_os_sec Tested OS
 * 
 * Linux
 *
 *\section example_sec Example Instantiation of the Module
 *
 * ./reaching_module --file conf/config_reaching_right_arm.ini --name /arm
 *
 * where conf/config_reaching_right_arm.ini is one of the files given in example in section \ref conf_file_sec
 * See the example in the \ref kinematic_simulator to see how to visualize the resulting motions in the simulator 
 * 
 * \author Micha Hersch 
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at \in src/lasaBodySchema/include/ReachingModuleThread.h
*/






#ifndef __REACHING_MODULE_H__
#define __REACHING_MODULE_H__

#define WITH_KIN_TREE


#include "KinematicTree.h"
#include "KChainBodySchema.h"


#include "ReachingGen.h"
#include "SpecializedPort.h" 
#include "yarp/os/all.h"
#include <yarp/os/RateThread.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

using namespace yarp::os;
using namespace yarp::dev;

/**
 * Thread imlementing the reaching controller, 
 * to ensure that timing are more or less respected 
 */


class ReachingThread: public RateThread {

protected:
    

    KinematicTree *body;
    KChainBodySchema *arm;        /**< the body schema describing the geometry of the manipulator */

    Reaching *reach;         /**< the class controlling the reaching behavior */
    //  char body_fname[80]; not used anymore
    
    bool active_mode;        /**< true of one wants the robot to be active */
    
    bool pointing;           /**<true if pointing is allowd */
    double time_last;        /**< last time position commands were sent */
    double time_now;         /**< current time */
    double original_time;    /**< application starting time */
    
    float time_granularity;  /**< the simulated period in [ms] between two integration step, i.e. dt */
    int nbDOFs;
    char partname[80];       /**< the name of the part the module is controlling */

    int *jointMapping;       /**< the mapping between joint numbers here and for 
                                the robot(one may not want to send all
                                the joints values to the robot e.g. virtual rotations) */
    int mappingSize;         /**< the size of this mapping , the number of joints here */
    
    Module *module;          /**< the module the thread belongs to */
    
    float *bodydata;         /**< buffer used to serialize the body schema */
    int body_data_size;      /**< size of bodydata */
    int new_schema;
    int listen_cnt;
    int simulation;          /**< if doing only simulation */

    // for recieving the input
    CartesianDataPort targetPort;             /**< the port that receives the target position */
    BodySchemaDataPort bodyPort;              /**< the port that receives updates of the body schema */

    JointAngleDataPort robotJointPort;

    // for sending the output
    BufferedPort<Bottle> vcControl_port;      /**< the port for sending the control parameters (gains) */
    BufferedPort<Bottle> vcFastCommand_port;  /**< the port for sending the joint angle commands */
    
    CommandPort cmdPort;




    // int cnt;// for debugging
 protected:
  bool listen();
  bool sendOutput();
  void sendAnglesFormatVC(joint_vec_t& angles);
  ConstString getName(const char *sub = 0);
    bool matchBodySchemaToRobot();//
    bool initMapping(Searchable& s);
    void setZeroGains();


 public:


  /**
   * Constructor
   * @param thread_period is given in milliseconds. Default value is 50 ms
   */
  ReachingThread(int thread_period=50);
  /**
   * Destructor
   */
  ~ReachingThread();
  
/**
   * Empty shell called at thread launching
   */
  bool threadInit();

  /**
   * cleans up allocated memory
   */
  void threadRelease();

  /**
   * initializes all the variables according to s (given by the command line)
   * @param s the module options, include: structure, orientation, granularity, name
   */
  bool init(Searchable &s);

  /**
   * the looping function. Checks for new target position commands, integrate the 
   * dynamical system and
   * sometimes sends out joint position command
   */
  void run();

  /**
   * Set method for the module variable
   */
  void setModule(Module *mod){module=mod;};
};



class ReachingModule : public Module{
 private: 
  ReachingThread m_thread;

 public:  
  ReachingModule();
  virtual ~ReachingModule(){cout<<"module deleted"<<endl;};
  virtual bool close();
  virtual bool open(Searchable &s);
  virtual bool updateModule(){return true;};
};


#endif
