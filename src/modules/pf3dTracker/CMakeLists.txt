# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale, Matteo Taiana, Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
project(pf3dTracker)

file(GLOB folder_source src/*.cpp)
file(GLOB folder_header include/iCub/*.hpp)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${folder_header} ${folder_source})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${YARP_LIBRARIES} )
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
