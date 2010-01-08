#include "TestMotorInterface.h"

const char* iCubMotorDriver::iCubPartName[NUM_ICUB_PARTS]={"torso","head","left_arm","right_arm","left_leg","right_leg"};

yarp::os::impl::String iCubMotorDriver::m_sRobot;