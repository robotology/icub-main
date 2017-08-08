/**
 * This file contains the definition of unique IDs for the body parts and the skin parts of the robot.
 * Moreover, it defines the association between each skin part and the corresponding body part and link number.
 *
 * By default the robot is iCub. 
 * If the compilation flag "NAO_SKIN" is set to ON then the robot is Nao.
 * 
 * \section intro_sec Description
 * 
 * \section tested_os_sec Tested OS
 * 
 * Windows, Linux
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
#include <string>
#include <vector>
#include <map>
#include <list>

namespace iCub
{

namespace skinDynLib
{

enum VerbosityLevel { NO_VERBOSE, VERBOSE, MORE_VERBOSE};

// DEFINE BODY PARTS AND SKIN PARTS OF ICUB ROBOT
#ifndef NAO_SKIN

// List of the parts of the iCub body
enum BodyPart {
    BODY_PART_UNKNOWN=0, 
    TORSO, HEAD, LEFT_ARM, RIGHT_ARM, LEFT_LEG, RIGHT_LEG, 
    BODY_PART_ALL, LOWER_BODY_PARTS, UPPER_BODY_PARTS, BODY_PART_SIZE
};
const std::string BodyPart_s[] = {
    "unknown_body_part", 
    "torso", "head", "left_arm", "right_arm", "left_leg", "right_leg", 
    "all_body_parts", "lower_body_parts", "upper_body_parts", "body_part_size"
};

// List of the parts composing the skin of iCub
enum SkinPart { 
    SKIN_PART_UNKNOWN=0, 
    SKIN_LEFT_HAND, SKIN_LEFT_FOREARM, SKIN_LEFT_UPPER_ARM, 
    SKIN_RIGHT_HAND, SKIN_RIGHT_FOREARM, SKIN_RIGHT_UPPER_ARM, 
    SKIN_FRONT_TORSO, LEFT_LEG_UPPER, LEFT_LEG_LOWER, LEFT_FOOT,
    RIGHT_LEG_UPPER, RIGHT_LEG_LOWER, RIGHT_FOOT,
    SKIN_PART_ALL, SKIN_PART_SIZE
};
const std::string SkinPart_s[] = {
    "unknown", 
    "l_hand", "l_forearm", "l_upper_arm", 
    "r_hand", "r_forearm", "r_upper_arm", 
    "chest",
    "l_upper_leg", "l_lower_leg", "l_foot",
    "r_upper_leg", "r_lower_leg", "r_foot",
    "all_skin_parts", "skin_part_size"
};

// *** Mapping from SKIN PARTs to BODY PARTs ***
// To represent this information we use a vector of struct{SkinPart; BodyPart;}
struct Skin_2_Body { SkinPart skin; BodyPart body; };
const Skin_2_Body SkinPart_2_BodyPart[SKIN_PART_SIZE] = {
    {SKIN_PART_UNKNOWN,     BODY_PART_UNKNOWN}, 
    {SKIN_LEFT_HAND,        LEFT_ARM}, 
    {SKIN_LEFT_FOREARM,     LEFT_ARM}, 
    {SKIN_LEFT_UPPER_ARM,   LEFT_ARM}, 
    {SKIN_RIGHT_HAND,       RIGHT_ARM}, 
    {SKIN_RIGHT_FOREARM,    RIGHT_ARM},
    {SKIN_RIGHT_UPPER_ARM,  RIGHT_ARM}, 
    {SKIN_FRONT_TORSO,      TORSO},
    {SKIN_PART_ALL,         BODY_PART_ALL}
};

// association between each skin part and the number of the link where it is mounted
struct Skin_2_Link{ SkinPart skin; int linkNum; };
const Skin_2_Link SkinPart_2_LinkNum[SKIN_PART_SIZE] = {
    {SKIN_PART_UNKNOWN,     -1}, 
    {SKIN_LEFT_HAND,         6}, 
    {SKIN_LEFT_FOREARM,      4}, 
    {SKIN_LEFT_UPPER_ARM,    2}, 
    {SKIN_RIGHT_HAND,        6}, 
    {SKIN_RIGHT_FOREARM,     4},
    {SKIN_RIGHT_UPPER_ARM,   2}, 
    {SKIN_FRONT_TORSO,       2},
    {SKIN_PART_ALL,         -1}
};
#endif

// DEFINE BODY PARTS AND SKIN PARTS OF NAO ROBOT
#ifdef NAO_SKIN
// List of the parts of the iCub body
enum BodyPart {
    BODY_PART_UNKNOWN=0, 
    HEAD, TORSO, LEFT_ARM, RIGHT_ARM, LEFT_LEG, RIGHT_LEG, 
    BODY_PART_ALL, BODY_PART_SIZE
};
const std::string BodyPart_s[] = {
    "unknown_body_part", 
    "head", "torso", "left_arm", "right_arm", "left_leg", "right_leg", 
    "all_body_parts", "body_part_size"
};

// List of the parts composing the skin of iCub
enum SkinPart { 
    SKIN_PART_UNKNOWN=0, 
    SKIN_LEFT_HAND, SKIN_LEFT_ARM, 
    SKIN_RIGHT_HAND, SKIN_RIGHT_ARM,
    SKIN_PART_ALL, SKIN_PART_SIZE
};
const std::string SkinPart_s[] = {
    "unknown_skin_part", 
    "skin_left_hand", "skin_left_arm",
    "skin_right_hand", "skin_right_arm",
    "all_skin_parts", "skin_part_size"
};

// association between each skin part and the corresponding body part
struct Skin_2_Body { SkinPart skin; BodyPart body; };
const Skin_2_Body SkinPart_2_BodyPart[SKIN_PART_SIZE] = {
    {SKIN_PART_UNKNOWN, BODY_PART_UNKNOWN}, 
    {SKIN_LEFT_HAND,    LEFT_ARM}, 
    {SKIN_LEFT_ARM,     LEFT_ARM}, 
    {SKIN_RIGHT_HAND,   RIGHT_ARM}, 
    {SKIN_RIGHT_ARM,    RIGHT_ARM},
    {SKIN_PART_ALL,     BODY_PART_ALL}
};

// association between each skin part and the number of the link where it is mounted
struct Skin_2_Link{ SkinPart skin; int linkNum; };
const Skin_2_Link SkinPart_2_LinkNum[SKIN_PART_SIZE] = {
    {SKIN_PART_UNKNOWN, -1}, 
    {SKIN_LEFT_HAND,    -1}, 
    {SKIN_LEFT_ARM,     -1}, 
    {SKIN_RIGHT_HAND,   -1}, 
    {SKIN_RIGHT_ARM,    -1},
    {SKIN_PART_ALL,     -1}
};
#endif

enum HandPart { 
    INDEX, MIDDLE, RING, LITTLE, THUMB, PALM, 
    ALL_HAND_PARTS, HAND_PART_SIZE
};
const std::string HandPart_s[] = {
    "index", "middle", "ring", "little", "thumb", "palm", 
    "all_hand_parts", "hand_part_size"
};

static const int ARM_FT_SENSOR_LINK_INDEX = 2;  // index of the link containing the F/T sensor in the iCub arms
static const int LEG_FT_SENSOR_LINK_INDEX = 1;  // index of the link containing the F/T sensor in the iCub legs

static const int ARM_DOF = 7;
static const int TORSO_DOF = 3;

/** 
* @ingroup skinDynLib 
*  
* Get the body part associated to the specified skin part.
* @param s the interested skin part
* @return the associated body part
*/
BodyPart getBodyPart(SkinPart s);

/** 
* @ingroup skinDynLib 
*  
* Get the list of skin parts associated to the specified body part.
* @param b the interested body part
* @return list of skin part associated to the specified body part
*/
std::vector<SkinPart> getSkinParts(BodyPart b);

/** 
* @ingroup skinDynLib 
*  
* Get the link number associated to the specified skin part.
* @param s the interested skin part
* @return the associated link number, -1 if the link number is not defined
*/
int getLinkNum(SkinPart s);

/** 
* @ingroup skinDynLib 
*  
* Get the SkinPart enum from the string version - essentially the opposite of SkinPart_s[]
* @param skinPartString the string of the skin part
* @return the SkinPart enum, SKIN_PART_UNKNOWN if the string cannot be resolved
*/
SkinPart getSkinPartFromString(const std::string skinPartString);



/**
* @ingroup skinDynLib 
* Converts a yarp::sig::Matrix to a yarp::sig::Vector.
* @param  m the matrix to convert
* @return   the matrix converted to a vector
**/
yarp::sig::Vector toVector(yarp::sig::Matrix m);

/**
* @ingroup skinDynLib 
* Retrieves a vector from a bottle.
* @param  b    the bottle to read the vector from
* @param  in   the start index
* @param  size the size of the vector (obviously, this have to hold: b.size()>in+size )
* @return      the vector extracted from the bottle
**/
yarp::sig::Vector vectorFromBottle(const yarp::os::Bottle b, int in, const int size);

/**
* Puts a Vector into a bottle, by cycling through its elements
* and adding them as doubles.
* @param  v the vector to put into the bottle
* @param  b the bottle to be filled with the vector
**/
void vectorIntoBottle(const yarp::sig::Vector v, yarp::os::Bottle &b);

/**
* Retrieves a matrix from a bottle.
* @param  b  the bottle
* @param  in the start index
* @param  r  the number of rows of the matrix
* @param  c  the number of cols of the matrix
* #return    the matrix extracted from the bottle
**/
yarp::sig::Matrix matrixFromBottle(const yarp::os::Bottle b, int in, const int r, const int c);

/**
* Puts a matrix into a bottle, by cycling through its elements
* and adding them as double
* @param  m the matrix to put into the bottle
* @param  b the bottle to be filled with the vector
**/
void      matrixIntoBottle(const yarp::sig::Matrix m, yarp::os::Bottle &b);

/**
* @ingroup skinDynLib 
* Returns a list of indexes corresponding to the values of vec that are equal to val.
* @param vec  the vector under evaluation
* @param val  the value to compare the vector to
* @return     a list of indexes
**/
inline std::list<unsigned int> vectorofIntEqualto(const std::vector<int> vec, const int val)
{
    std::list<unsigned int> res;
    for (size_t i = 0; i < vec.size(); i++)
    {
        if (vec[i]==val)
        {
            res.push_back(i);
        }
    }
    return res;
};

/**
 * \brief Macro to deprecate functions and methods
 *
 * see https://blog.samat.io/2017/02/27/Deprecating-functions-and-methods-in-Cplusplus/
 */
// C++14
#if __cplusplus >= 201402L
        #if defined(__has_cpp_attribute)
    #if __has_cpp_attribute(deprecated)
      #define SKINDYNLIB_DEPRECATED(msg) [[deprecated(msg)]]
    #endif
  #endif
// Earlier standards
#else
#if defined(__GNUC__) || defined(__clang__)
#define SKINDYNLIB_DEPRECATED(msg) __attribute__((deprecated(msg)))
#elif defined(_MSC_VER)
    #define SKINDYNLIB_DEPRECATED(msg) __declspec(deprecated(msg))
  #else
    #define SKINDYNLIB_DEPRECATED(msg)
#endif
#endif

}

}//end namespace

#endif


