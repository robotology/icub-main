# Copyright: (C) 2010 RobotCub Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME actionPrimitives)
project(${PROJECTNAME})

set(folder_source src/actionPrimitives.cpp)
set(folder_header include/iCub/action/actionPrimitives.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${perceptiveModels_INCLUDE_DIRS})

add_library(${PROJECTNAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECT_NAME} perceptiveModels ${YARP_LIBRARIES})

icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                   DEPENDS perceptiveModels
                                   DESTINATION include/iCub/action
                                   FILES ${folder_header})


