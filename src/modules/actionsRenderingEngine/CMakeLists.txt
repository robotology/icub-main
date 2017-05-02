# Copyright: 2011 RobotCub Consortium
# Author: Carlo Ciliberto
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

project(actionsRenderingEngine)

find_package(IPOPT REQUIRED)

set(folder_source src/main.cpp
                  src/utils.cpp
                  src/pointing_far.cpp
                  src/MotorThread.cpp
                  src/VisuoThread.cpp)

set(folder_header include/iCub/utils.h 
                  include/iCub/pointing_far.h 
                  include/iCub/VisuoThread.h
                  include/iCub/MotorThread.h)
                  
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${iKin_INCLUDE_DIRS} ${actionPrimitives_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${IPOPT_INCLUDE_DIRS})

add_definitions(${IPOPT_DEFINITIONS} -D_USE_MATH_DEFINES)
add_executable(${PROJECT_NAME} ${folder_header} ${folder_source})

set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS " ${IPOPT_LINK_FLAGS}")
target_link_libraries(${PROJECT_NAME} iKin actionPrimitives ${YARP_LIBRARIES}
                                      ${OpenCV_LIBRARIES} ${IPOPT_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

