# Copyright: 2010 RobotCub Consortium
# Author: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

project(cartesianInterfaceControl)
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} iKin ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

