# Copyright: (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME calibration)
project(${PROJECTNAME})

set(folder_source src/calibReference.cpp)
set(folder_header include/iCub/calibration/calibReference.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${IPOPT_INCLUDE_DIRS}
                    ${ctrlLib_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

add_library(${PROJECTNAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECTNAME} ${IPOPT_LIBRARIES}
                                     ctrlLib
                                     ${YARP_LIBRARIES})
                 
icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                   EXTERNAL_INCLUDE_DIRS ${IPOPT_INCLUDE_DIRS}
                                   DEPENDS ctrlLib
                                   DESTINATION include/iCub/calibration
                                   FILES ${folder_header})


