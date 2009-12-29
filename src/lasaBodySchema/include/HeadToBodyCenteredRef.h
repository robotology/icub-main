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
 * \defgroup icub_head_transfo  head_transfo  
 * 
 * Converts position from eyes-centered to body-centered frames of reference
 *
 * \section intro_sec Description
 * This module converts positions from a frame of reference centered on one joint to a 
 * frame of reference centered on another joint. It has been used mainly for converting from
 * eyes-centered to body-centered frames of reference.
 * It can receive updated version of the body schema from the network.
 *
 * \section lib_sec Libraries
 * yarp_os
 *
 * \section parameters_sec Parameters
 * - --structure <struct_file> : the file describing the kinematic structure of the robot. It is
 * the  The format of this file is described in section \ref in_file_sec. It is the same file 
 * that is given as parameter of
 * the reaching_module, the body_schema_module and the kinematic_simulator
 * - --from <head_base> : the name of the most proximal joint of the head
 * - --to <head_end> : the name of the most distal joint of the head
 * - --name <basename> : port prefix
 * - --file <config_file> : configuration file where you can specify the above parameters
 *
 * \section portsa_sec Ports Accessed
 * none directly in the code 
 *
 * \section portsc_sec Ports Created 
 * Input ports:
 * -- <basename>/proprio_head:i this ports receives the joint angle values for the head in degrees.
 * It a list of doubles specifiy the angles, from the proximal to the distal joints.
 * Typically it gets its input from /icub/head/state:o
 * - <basename>/position:i This port gets a 3d position in a head-centered frame of reference.
 * The units is the
 * same as the one used in the <struct_file> typically [mm]. If you want also to specify the
 * orientation, it is only one line to change in the code. This position will converted to a
 * body-centered frame of reference
 * - <basename>/body_schema:i this port can recieve updates of the body schema. The format is
 * a string of doubles, as described by the ArticulatedTree::Deserialize method
 * - <basename>/cmd:i a port where commands can be sent. For now only the quit command is 
 * implemented
 *
 * Output ports:
 *
 * - <basename>/position:i this port output the position in a body-centered frame of reference
 *
 *  \section in_files_sec Input Data Files
 * The file <struct_file> specifes the kinematic structure 
 * of the manipulator. Typically, this file describes the kinematic structure of the entire robot
 *  The robot can be described by an ArticulatedTree , having (for example)
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
 * In the example above r_sfe is the name of the proximal joint. The <Position> tag refers to
 * the position of the joint with resect to the previous joint, when in the zero joint position.
 * Similarly
 * the <Axis> tag refers the the rotation axis, when all previous joints are in the zero position. 
 * <Range> provides the joint angle boundaries in degrees, and <Children> announce the next link.
 * The tip of the manipulator is specified as an additional joint, but it has no <Axis> tag, only a
 * <Position> tag 

 *\section out_data_sec Output Data Files
 * None

 *\section conf_file_sec Configuration Files
 * All the parameters can be written in a file given by the --file option on the command line. Here is
 * an example for reaching with the right arm
 * \code
 * structure ./conf/icub_head_right_arm_tree_safe.xml 
 * from neck_tilt
 * to eyes 
 * \endcode
 *
 *\section tested_os_sec Tested OS
 * 
 * Linux
 *
 *\section example_sec Example Instantiation of the Module
 *
 * \code
 * ./head_transfo --file conf/config_head2body.ini --name /micha/transfo
 * \endcode
 * where conf/config_head2body.ini is the file given in example in section \ref conf_file_sec
 * 
 * \author Micha Hersch
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at src/lasaBodySchema/include/HeadToBodyCenteredRef.h 
 */

#ifndef __HEAD_TO_BODY_CENTERED_REF_H__
#define __HEAD_TO_BODY_CENTERED_REF_H__

#include "KinematicTree.h"
#include "KChainOrientationBodySchema.h"
#include "SpecializedPort.h"
#include "yarp/os/all.h"


class HeadToBodyCenteredRef:public Module{

 protected:

  KinematicTree *body;
  KChainBodySchema *head;
 
  CartesianDataPort in;
  CartesianDataPort out;
  JointAngleDataPort headProprio;
  BodySchemaDataPort bodyPort;
  CommandPort cmdPort;


  joint_vec_t proprio;
  cart_vec_t visionPosBody;
  cart_vec_t visionPosHead;

  float *bodydata;         /**< buffer used to serialize the body schema */
  int body_data_size;      /**< size of bodydata */
  
 public:
  HeadToBodyCenteredRef();
  virtual   ~HeadToBodyCenteredRef(){};
  virtual bool open(Searchable& config);
  virtual bool close();
  virtual bool updateModule();
};
#endif
