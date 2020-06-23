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
                  src/outliersDetection.cpp
                  src/clustering.cpp)

set(folder_header include/iCub/ctrl/math.h
                  include/iCub/ctrl/filters.h
                  include/iCub/ctrl/kalman.h
                  include/iCub/ctrl/pids.h
                  include/iCub/ctrl/tuning.h
                  include/iCub/ctrl/adaptWinPolyEstimator.h
                  include/iCub/ctrl/minJerkCtrl.h
                  include/iCub/ctrl/optimalControl.h
                  include/iCub/ctrl/neuralNetworks.h
                  include/iCub/ctrl/outliersDetection.h
                  include/iCub/ctrl/clustering.h)

if(ICUB_USE_GSL)
  set(folder_source ${folder_source} src/functionEncoder.cpp)
  set(folder_header ${folder_header} include/iCub/ctrl/functionEncoder.h)
  include_directories(${GSL_INCLUDE_DIRS})
endif()

add_library(${PROJECT_NAME} ${folder_source} ${folder_header})
add_library(ICUB::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

set_target_properties(${PROJECT_NAME} PROPERTIES
                                      PUBLIC_HEADER "${folder_header}")

target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})

set(CTRLLIB_DEPENDENCIES YARP_os
                         YARP_sig
                         YARP_dev
                         YARP_math)
if(ICUB_USE_GSL)
  target_link_libraries(${PROJECT_NAME} ${GSL_LIBRARIES})
  list(APPEND CTRLLIB_DEPENDENCIES YARP_gsl GSL)
endif()

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        LIBRARY
            DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        PUBLIC_HEADER
            DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/iCub/ctrl")


icub_install_basic_package_files(${PROJECT_NAME}
                                 DEPENDENCIES ${CTRLLIB_DEPENDENCIES})
