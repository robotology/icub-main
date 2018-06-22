# Copyright (C) 2010 RobotCub Consortium
# Author: Matteo Fumagalli
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(gravityCompensator)
set(folder_source main.cpp gravityThread.cpp)
set(folder_header gravityThread.h)

add_executable(${PROJECT_NAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECT_NAME} skinDynLib iDyn ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
