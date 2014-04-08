# Copyright: (C) 2010 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME skinWrapper)
PROJECT(${PROJECTNAME})

PREPARE_DEVICE(${PROJECTNAME} TYPE ${PROJECTNAME} INCLUDE ${PROJECTNAME}.h)

IF (NOT SKIP_${PROJECTNAME})
  INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/libraries/icubmod/debugStream)
  INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/libraries/icubmod/analogServer)
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS} ${iCubDev_INCLUDE_DIRS})

  yarp_add_plugin(${PROJECTNAME} ${PROJECTNAME}.cpp ${PROJECTNAME}.h ${CMAKE_SOURCE_DIR}/src/libraries/icubmod/analogServer/analogServer.cpp ${CMAKE_SOURCE_DIR}/src/libraries/icubmod/analogServer/analogServer.h)
  TARGET_LINK_LIBRARIES(${PROJECTNAME}  debugStream ${YARP_LIBRARIES} ${ACE_LIBRARIES})
  icub_export_library(${PROJECTNAME})
ENDIF (NOT SKIP_${PROJECTNAME})
