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
#ifndef __YARP_SIMULATOR_H__
#define __YARP_SIMULATOR_H__



#include "GLUTApplication.h"
#include "EmbodiedArticulatedTree.h"
#include "SpecializedPort.h"
#include "Shape.h"
#include <yarp/os/all.h>
#include "BodySchema.h"


/**
 * @ingroup icub_module
 *
 * \defgroup icub_kinematic_simulator  kinematic_simulator  
 * 
 * An openGL kinematic simulator of the robot
 *
 * \section intro_sec Description
 *
 * This is an openGL kinematic simulator. It simulates rotational joint robots with arbitrary
 * geometries. It takes a input an xml file describing the geometry of the robot. It gets the
 * joint angles values through a port and draws the corresponding position of the robot. It can read updated
 * versions of the body schema through a port. It display both the updated version and the original version
 * of the robot, which is useful to visualize the effect of body schema learning. It can
 * also get the position (and orientation) of a target and also draws it. Using the keyboard, one
 * can also move the target and send the resulting position on a port.
 *
 * \section lib_sec Libraries
 *  - yarp_os
 *  - glut (openGL)
 *  
 * \section parameters_sec Parameters
 * The call to this module is done as follows:
 
 * ./simulator <struct_file> <shape_file> [--nomapping]
 *  - <struct_file> an xml file describing the kinematic structure of the robot. The format 
 * of the file is described in section \ref in_file_sec. It is the same file that is given as parameter of
 * the reaching module (cf ReachingModuleThread.h), the body_schema_module (cf BodySchemaLearning.h), 
 * and the head_transfo module (cf HeadToBodyCenteredRef.h).  
 *  - <shape_file> an xml file describing the shape attached to each limb. The format 
 * of the file is described in section \ref in_file_sec.
 *
 *  
 * \section in_file_sec Input Data File
 *
 * The file <struct_file> specifes the kinematic structure
 * This section describes the format of the file <struct_file> specifying the kinematic structure 
 * of the manipulator. The robot can be described by a "kinematic tree", having (for example)
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
 * The <shape_file> describe the shapes to be attached to head joint. There are three kinds of possible
 * shape: Capsules, Parallelipipeds and Spheres. Here is an example of shape file: 
 * \code
 * <Body>
 * <Shape> 
 *  <Name> r_sfe </Name>
 *  <Type> Capsule </Type>
 *  <Params> 
 *    <Position> 0 0 0 </Position>
 *    <Radius> 30 </Radius> 
 *  </Params>
 * </Shape>
 * <Shape>
 *  <Name> r_wrist2 </Name>
 *  <Type> Parallelipiped </Type>
 *  <Params>
 *    <Position> 0 0 -30 </Position> 
 *    <Size> 10 -100.3 60 </Size>
 *  </Params>
 * </Shape>
 * <Shape>
 *  <Name> gaze </Name>
 *  <Type> Sphere </Type>
 *  <Params>
 *    <Position> 0 0 60 </Position> 
 *    <Radius> 20 </Radius> 
 *  </Params>
 * </Shape>
 *</Body>
 *\endcode
 * - <Name> refers to the name defined in <struct_file>
 * - <Position> is the position of the Capsule extremity,
 * Sphere center or Parallelipiped center with respect to the frame of reference of the corresponding limb. 
 * - <Radius> refers to the Capsule or Sphere radius
 * - <Size> refers to the size of Parallelipiped in the frame of reference of the corresponding limb. 

 * \section portsa_sec Ports Accessed
 * none directly in the code  
 * \section portsc_sec Ports Created
 *
 * Input ports:
 * - /simulator/arm:i to get the joint angles of the arm (from r_sfe to r_wrist2)
 * - /simulator/head:i to get the joint angles of the head (from head_tilt to eye_tilt)
 * - /simulator/target:i to get the target position in a torso frame of reference as 'x y z [rotx roty rotz]'
 * where rotx roty rotz describe the target orientation with the 3 independent quaternion component  
 * - /simulator/body_schema:i to get updated versions of the body schema. 
 *
 * Output ports:
 * - /simulator/target:o to send the target as 'x y z' in a torso frame of reference.
 * - /simulator/cmd:o to send quit commands to other modules. 
 *
 * \section conf_file_sec Configuration Files
 *  This module does not accept configuration files
 *
 * \section tested_os_sec Tested OS
 *
 * linux and in a previous version windows
 *
 * \section example_sec Example Instantiation of the Module
 *  For reaching to random target with the reaching_module and seeing the result on the simulator
 *  \code
 *   xterm -T "reaching arm" -e "./reaching_module --file conf/config_reaching_right_arm.ini --name /arm 
 *   --orientation 1 --simulation" &
 *   xterm -T "simulator" -e "./simulator conf/icub_head_right_arm_tree.xml conf/icub_head_right_arm_shape.xml" &
 *  sleep 2
 *  yarp connect /arm/vc_command:o /simulator/arm:i
 *  yarp connect  /arm/target:i /simulator/target:i
 *  yarp connect /simulator/cmd:o arm/cmd:i
 * \endcode                        
 *  where the configuration files are available in the repository
 * \section inter_sec User Interface
 * You can:
 * - move the target my using the arrows and pageUp, pageDown key.
 * - send the target position
 *  to the /simulator/target:o port by pressing the 's' key. 
 * - press the 'Esc' key, to send a quit signal to the simulator/cmd:o port.
 * - change the viewing parameters by moving the mouse while pressing one of the three buttons
 *  
 *
 *
 * \author: Micha Hersch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at src/lasaBodySchema/include/KinematicSimulator.h 
 */

#define TWO_BODIES


class KinematicSimulator: public GLUTBaseApplication{


protected:

    EmbodiedKinematicTree *body;
    KinematicChain *arm;
    KinematicChain *head;

#ifdef TWO_BODIES
    EmbodiedKinematicTree *body2;
    KinematicChain *arm2;
    KinematicChain *head2;
    cart_vec_t display_trans;
#endif
    
    float *angles;
    float *arm_angles;
    float *head_angles;
  //  float *min_angles;
  // float *max_angles;

    int body_size;
    int arm_size;
    int head_size;
    int target_info;
    
    cart_vec_t tarpos;

    char tree_file[80];
    char shape_file[80];
 
    char port_base_name[80];
    
    JointAngleDataPort proprio_arm;//("/proprioListener");
    JointAngleDataPort proprio_head;

    CartesianDataPort targetIn;
    CartesianDataPort targetOut;
    
    CommandPort cmdPort;
 
    BodySchemaDataPort bodySchema;
    float *body_data;
    int body_data_size;

    bool m_bProprioActive;
    int mapping;



public:
    KinematicSimulator();
    KinematicSimulator(int argc, char **argv);
    virtual ~KinematicSimulator();
    virtual void RenderTarget();
    virtual void Render();
    virtual void InputSpecialKey(int key,int x, int y);
    virtual void InputNormalKey(unsigned char key,int x, int y);
    virtual void Init();
    virtual void OnIdle();
    virtual void ClosePorts();  
    void InitBody();
    void InitOpenGL();
    void ListenToRobot();
    void Rearrange(float *angles);

 };


#endif
