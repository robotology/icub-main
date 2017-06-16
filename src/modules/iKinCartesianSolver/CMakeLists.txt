# Copyright: 2010 RobotCub Consortium
# Author: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

project(iKinCartesianSolver)

set(folder_source main.cpp)

include_directories(${iKin_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} ${folder_source})
target_link_libraries(${PROJECT_NAME} iKin ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)


