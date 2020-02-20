# Copyright: 2011 RobotCub Consortium
# Author: Carlo Ciliberto
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

project(actionsRenderingEngine)

find_package(YARP COMPONENTS os sig dev math cv)
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

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${IPOPT_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${folder_header} ${folder_source})
target_compile_definitions(${PROJECT_NAME} PRIVATE ${IPOPT_DEFINITIONS} _USE_MATH_DEFINES)

set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS " ${IPOPT_LINK_FLAGS}")
target_link_libraries(${PROJECT_NAME} iKin actionPrimitives ${YARP_LIBRARIES}
                                      ${OpenCV_LIBRARIES} ${IPOPT_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

