# Copyright: (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME optimization)
project(${PROJECTNAME})

set(folder_source src/algorithms.cpp
                  src/calibReference.cpp
                  src/affinity.cpp
                  src/neuralNetworks.cpp)
set(folder_header include/iCub/optimization/algorithms.h
                  include/iCub/optimization/matrixTransformation.h
                  include/iCub/optimization/calibReference.h
                  include/iCub/optimization/affinity.h
                  include/iCub/optimization/neuralNetworks.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${PROJECT_SOURCE_DIR}/include                    
                    ${ctrlLib_INCLUDE_DIRS}
                    ${IPOPT_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

add_definitions(${IPOPT_DEFINITIONS})
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${IPOPT_LINKER_FLAGS}")

add_library(${PROJECTNAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECTNAME} ctrlLib
                                     ${IPOPT_LIBRARIES}
                                     ${YARP_LIBRARIES})
                 
icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                   DEPENDS ctrlLib
                                   DESTINATION include/iCub/optimization
                                   FILES ${folder_header})


