# Copyright: (C) 2010 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME analogServer)
#PROJECT(${PROJECTNAME})

yarp_prepare_plugin(analogServer CATEGORY device TYPE yarp::dev::AnalogServer INCLUDE ${PROJECTNAME}.h)

IF (NOT SKIP_${PROJECTNAME})
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS} ${iCubDev_INCLUDE_DIRS} )

  yarp_add_plugin(${PROJECTNAME} ${PROJECTNAME}.cpp ${PROJECTNAME}.h)
  TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES} ${ACE_LIBRARIES})
  icub_export_plugin(${PROJECTNAME})
ENDIF (NOT SKIP_${PROJECTNAME})

