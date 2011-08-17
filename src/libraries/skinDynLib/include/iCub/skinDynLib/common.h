/**
 * \defgroup iCubCommons
 *  
 * 
 * 
 * \section intro_sec Description
 * 
 * \section tested_os_sec Tested OS
 * 
 * Windows
 *
 * \author Andrea Del Prete
 * 
 * Copyright (C) 2011 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * 
 **/ 

#ifndef __COMMON_H__
#define __COMMON_H__

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

namespace iCub
{

namespace skinDynLib
{

// print a matrix nicely
void printMatrix(yarp::sig::Matrix &m, std::string description="", unsigned int precision=3);

// print a vector nicely
void printVector(yarp::sig::Vector &v, std::string description="", unsigned int precision=3);

enum VerbosityLevel { NO_VERBOSE, VERBOSE, MORE_VERBOSE};

enum BodyPart {UNKNOWN_BODY_PART, HEAD, TORSO, LEFT_ARM, RIGHT_ARM, LEFT_LEG, RIGHT_LEG, ALL_BODY_PARTS, BODY_PART_SIZE};
const std::string BodyPart_s[] = {"unknown_body_part", "head", "torso", "left_arm", "right_arm", "left_leg", "right_leg", "all_body_parts", "body_part_size"};

enum SkinPart { UNKNOWN_SKIN_PART, HAND, FOREARM_LOWER, FOREARM_UPPER, ARM_INTERNAL, ARM_EXTERNAL, 
    ARM_LOWER, ARM_UPPER, ALL_SKIN_PARTS, SKIN_PART_SIZE};
const std::string SkinPart_s[] = {"unknown_skin_part", "hand", "forearm_lower", "forearm_upper", "arm_internal", 
    "arm_external", "arm_lower", "arm_upper", "all_skin_parts", "skin_part_size"};

enum HandPart { INDEX, MIDDLE, RING, LITTLE, THUMB, PALM, ALL_HAND_PARTS, HAND_PART_SIZE};
const std::string HandPart_s[] = {"index", "middle", "ring", "little", "thumb", "palm", "all_hand_parts", "hand_part_size"};

static const int ARM_FT_SENSOR_LINK_INDEX = 2;  // index of the link containing the F/T sensor in the iCub arms
static const int LEG_FT_SENSOR_LINK_INDEX = 1;  // index of the link containing the F/T sensor in the iCub legs
static const int ARM_DOF = 7;
}

}//end namespace

#endif


