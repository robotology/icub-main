# Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
# Copyright (C) 2006-2010 RobotCub Consortium
# All rights reserved.
#
# This software may be modified and distributed under the terms
# of the BSD-3-Clause license. See the accompanying LICENSE file for
# details.

project(ctrlLib)

set(folder_source src/math.cpp
                  src/filters.cpp
                  src/kalman.cpp
                  src/pids.cpp
                  src/tuning.cpp
                  src/adaptWinPolyEstimator.cpp
                  src/minJerkCtrl.cpp
                  src/optimalControl.cpp
                  src/neuralNetworks.cpp
                  src/outliersDetection.cpp)

set(folder_header include/iCub/ctrl/math.h
                  include/iCub/ctrl/filters.h
                  include/iCub/ctrl/kalman.h
                  include/iCub/ctrl/pids.h
                  include/iCub/ctrl/tuning.h
                  include/iCub/ctrl/adaptWinPolyEstimator.h
                  include/iCub/ctrl/minJerkCtrl.h
                  include/iCub/ctrl/optimalControl.h
                  include/iCub/ctrl/neuralNetworks.h
                  include/iCub/ctrl/outliersDetection.h)

include_directories(${PROJECT_SOURCE_DIR}/include ${YARP_INCLUDE_DIRS})

if(ICUB_USE_GSL)
  set(folder_source ${folder_source} src/functionEncoder.cpp)
  set(folder_header ${folder_header} include/iCub/ctrl/functionEncoder.h)
  include_directories(${GSL_INCLUDE_DIRS})
endif()

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

add_definitions(-D_USE_MATH_DEFINES)
add_library(${PROJECT_NAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})

if(ICUB_USE_GSL)
  target_link_libraries(${PROJECT_NAME} ${GSL_LIBRARIES})
endif()

icub_export_library(${PROJECT_NAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                    DESTINATION include/iCub/ctrl
                                    FILES ${folder_header})

