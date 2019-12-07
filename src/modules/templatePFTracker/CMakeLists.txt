# Copyright: 2009 RobotCub Consortium
# Author: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

project(templatePFTracker)

find_package(YARP COMPONENTS os sig cv)

file(GLOB folder_header include/iCub/*.h)
file(GLOB folder_source src/*.cpp src/*.cc src/*.c)

include_directories(${PROJECT_SOURCE_DIR}/include ${GSL_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} ${folder_header} ${folder_source})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${GSL_LIBRARIES} ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

