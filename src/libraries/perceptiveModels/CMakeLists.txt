# Copyright: (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(perceptiveModels)

set(folder_source src/ports.cpp
                  src/sensors.cpp
                  src/nodes.cpp
                  src/models.cpp
                  src/springyFingers.cpp
                  src/tactileFingers.cpp)

set(folder_header_private include/iCub/perception/private/ports.h
                          include/iCub/perception/private/models.h)

set(folder_header include/iCub/perception/sensors.h
                  include/iCub/perception/nodes.h
                  include/iCub/perception/models.h
                  include/iCub/perception/springyFingers.h
                  include/iCub/perception/tactileFingers.h)

add_library(${PROJECT_NAME} ${folder_source} ${folder_header_private} ${folder_header})

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

target_link_libraries(${PROJECT_NAME} ctrlLib
                                      learningMachine
                                      ${YARP_LIBRARIES})

icub_export_library(${PROJECT_NAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                    DEPENDS ctrlLib learningMachine
                                    DESTINATION include/iCub/perception
                                    FILES ${folder_header})

