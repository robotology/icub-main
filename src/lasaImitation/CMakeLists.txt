# Copyright: (C) 2009 RobotCub Consortium
# Authors: Eric Sauser
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)

PROJECT (lasaImitation)

# Check for the minimal set of packages required for the global project.
# If we include them here, sub-projects can be configured a bit faster.

FIND_PACKAGE(YARP REQUIRED)

SET(NESTED_BUILD TRUE)
SET(PM_PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

SET(BUILD_MOTIONSSENSORS FALSE CACHE BOOL "Do you have the XSens and 5DT drivers?")

SET(SUBPROJECTS_LIST MathLib
                     StdTools
                     YarpTools
                     UDPConnection
                     GLTools
                     MyDefaultModule
                     MultipleMiceDriver
                     MultipleMiceExample
                     DataStreamerModule
                     TouchControllerModule
                     VelocityControllerExt
                     GaussianMixtureModelModule
                     RobotControllerModule
                     ImitationApplicationModule
)

IF (BUILD_MOTIONSSENSORS)
  SET(SUBPROJECTS_LIST ${SUBPROJECTS_LIST} YarpedMotionSensors)
ENDIF (BUILD_MOTIONSSENSORS)

FOREACH(SUBPROJECT_NAME ${SUBPROJECTS_LIST})
  SET(${SUBPROJECT_NAME}_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${SUBPROJECT_NAME})
ENDFOREACH(SUBPROJECT_NAME)

FOREACH(SUBPROJECT_NAME ${SUBPROJECTS_LIST})
  ADD_SUBDIRECTORY(${SUBPROJECT_NAME})
ENDFOREACH(SUBPROJECT_NAME)


