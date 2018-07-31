# Copyright: (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(optimization)

set(folder_source src/algorithms.cpp
                  src/calibReference.cpp
                  src/affinity.cpp
                  src/neuralNetworks.cpp)
set(folder_header include/iCub/optimization/algorithms.h
                  include/iCub/optimization/matrixTransformation.h
                  include/iCub/optimization/calibReference.h
                  include/iCub/optimization/affinity.h
                  include/iCub/optimization/neuralNetworks.h)

# import math symbols from standard cmath
add_definitions(${IPOPT_DEFINITIONS} -D_USE_MATH_DEFINES)
add_library(${PROJECT_NAME} ${folder_source} ${folder_header})

target_include_directories(${PROJECT_NAME} PUBLIC  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
                                                   "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>"
                                           PRIVATE ${IPOPT_INCLUDE_DIRS})

set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS " ${IPOPT_LINK_FLAGS}")
target_link_libraries(${PROJECT_NAME} ctrlLib
                                      ${IPOPT_LIBRARIES}
                                      ${YARP_LIBRARIES})
                 
icub_export_library(${PROJECT_NAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                    DEPENDS ctrlLib
                                    DESTINATION include/iCub/optimization
                                    FILES ${folder_header})


