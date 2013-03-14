# Copyright: (C) 2010 RobotCub Consortium
# Authors: Arjan Gijsberts
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)

set(projectNAME lmExample)

set(LMLIB_DIRECT lmlibdirect)
set(LMLIB_INDIRECT lmlibindirect)
set(LMLIB_PORTABLE lmlibportable)

project(${projectNAME})

find_package(YARP)
# uncomment later
#find_package(ICUB)

set(LEARNINGMACHINE_DIR "..")

#LINK_LIBRARIES(learningMachine)
#include_directories(${learningMachine_INCLUDE_DIRS})
# swap later with previous line
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${LEARNINGMACHINE_DIR}/include)
link_directories(${LINK_DIRECTORIES} ${LEARNINGMACHINE_DIR}/standalone)


add_executable(${LMLIB_DIRECT} ${LMLIB_DIRECT}.cpp)
add_executable(${LMLIB_INDIRECT} ${LMLIB_INDIRECT}.cpp)
add_executable(${LMLIB_PORTABLE} ${LMLIB_PORTABLE}.cpp)

# remove later
target_link_libraries(${LMLIB_DIRECT} learningMachine ${YARP_LIBRARIES})
target_link_libraries(${LMLIB_INDIRECT} learningMachine ${YARP_LIBRARIES})
target_link_libraries(${LMLIB_PORTABLE} learningMachine ${YARP_LIBRARIES})
