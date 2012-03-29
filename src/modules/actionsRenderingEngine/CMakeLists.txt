# Copyright: 2011 RobotCub Consortium
# Author: Carlo Ciliberto
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

set(PROJECTNAME actionsRenderingEngine)
project(${PROJECTNAME})

set(folder_source src/main.cpp
                  src/utils.cpp
                  src/MotorThread.cpp
                  src/VisuoThread.cpp)

set(folder_header include/iCub/utils.h 
                  include/iCub/VisuoThread.h
                  include/iCub/MotorThread.h)
                  
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${PROJECT_SOURCE_DIR}/include ${YARP_INCLUDE_DIRS} ${iKin_INCLUDE_DIRS} ${actionPrimitives_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${folder_header} ${folder_source})
target_link_libraries(${PROJECTNAME} iKin actionPrimitives ${OpenCV_LIBRARIES} ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)
