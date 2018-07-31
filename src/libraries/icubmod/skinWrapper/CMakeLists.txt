# Copyright: (C) 2010 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME skinWrapper)
PROJECT(${PROJECTNAME})

yarp_prepare_plugin(${PROJECTNAME} CATEGORY device TYPE skinWrapper INCLUDE ${PROJECTNAME}.h)

IF (NOT SKIP_${PROJECTNAME})
  INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/libraries/icubmod/analogServer)
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${iCubDev_INCLUDE_DIRS})

  yarp_add_plugin(${PROJECTNAME} ${PROJECTNAME}.cpp ${PROJECTNAME}.h)
  TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES} ${ACE_LIBRARIES})
  icub_export_plugin(${PROJECTNAME})
  yarp_install(FILES skinWrapper.ini  DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})
ENDIF (NOT SKIP_${PROJECTNAME})
