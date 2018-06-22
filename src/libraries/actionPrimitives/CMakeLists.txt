# Copyright: (C) 2010 RobotCub Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(actionPrimitives)

set(folder_source src/actionPrimitives.cpp)
set(folder_header include/iCub/action/actionPrimitives.h)

add_library(${PROJECT_NAME} ${folder_source} ${folder_header})

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

target_link_libraries(${PROJECT_NAME} perceptiveModels ${YARP_LIBRARIES})

icub_export_library(${PROJECT_NAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                    DEPENDS perceptiveModels
                                    DESTINATION include/iCub/action
                                    FILES ${folder_header})


