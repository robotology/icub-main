# Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# ========================
# MAIN ICUB TARGET
# ========================

# Dummy target
add_library(${ROOT_PROJECT_NAME} INTERFACE)

install(
    TARGETS ${ROOT_PROJECT_NAME}
    EXPORT ${ROOT_PROJECT_NAME}Export)

set(ICUB_DEFAULT_FIND_COMPONENTS iCubDev
                                 ctrlLib
                                 skinDynLib
                                 iKin
                                 iDyn)

if(TARGET ICUB::learningMachine)
  list(APPEND ICUB_DEFAULT_FIND_COMPONENTS learningMachine perceptiveModels actionPrimitives)
endif()

if(TARGET ICUB::optimization)
  list(APPEND ICUB_DEFAULT_FIND_COMPONENTS optimization)
endif()
set(ICUB_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")
install_basic_package_files(ICUB
    VERSION ${${ROOT_PROJECT_NAME}_VERSION}
    COMPATIBILITY AnyNewerVersion
    EXTRA_PATH_VARS_SUFFIX PREFIX
    CONFIG_TEMPLATE  ${CMAKE_SOURCE_DIR}/conf/ICUBConfig.cmake.in
    EXPORT ${ROOT_PROJECT_NAME}Export
    DEPENDENCIES "YCM  ${YCM_REQUIRED_VERSION}  REQUIRED"
                 "YARP ${YARP_REQUIRED_VERSION} REQUIRED COMPONENTS os conf sig dev math gsl"
                 ${ICUB_DEFAULT_FIND_COMPONENTS}
    NAMESPACE ${ROOT_PROJECT_NAME}::
    INCLUDE_FILE ${CMAKE_SOURCE_DIR}/conf/ExtraPackageConfigVars.cmake.in)
