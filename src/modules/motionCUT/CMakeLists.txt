# Copyright: 2010 RobotCub Consortium
# Author: Carlo Ciliberto, Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

set(PROJECTNAME motionCUT)
project(${PROJECTNAME})

set(folder_header main.h)
#if(OpenCV_VERSION_MAJOR GREATER 2 OR OpenCV_VERSION_MAJOR EQUAL 2 AND OpenCV_VERSION_MINOR GREATER 2)
#    set(folder_source main_cv_cpp.cpp)
#else()
    set(folder_source main_cv_c.cpp)
#endif()

source_group("Header Files" FILES ${folder_header})
source_group("Source Files" FILES ${folder_source})

include_directories(${OpenCV_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${folder_header} ${folder_source})
target_link_libraries(${PROJECTNAME} ${OpenCV_LIBRARIES} ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)

