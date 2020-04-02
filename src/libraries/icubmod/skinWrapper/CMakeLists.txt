# Copyright: (C) 2010 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(skinWrapper)

yarp_prepare_plugin(${PROJECT_NAME} CATEGORY device TYPE skinWrapper INCLUDE ${PROJECT_NAME}.h)

IF (NOT SKIP_${PROJECT_NAME})
  INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/libraries/icubmod/analogServer)
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

  yarp_add_plugin(${PROJECT_NAME} ${PROJECT_NAME}.cpp ${PROJECT_NAME}.h)
  TARGET_LINK_LIBRARIES(${PROJECT_NAME} iCubDev ACE::ACE)
  icub_export_plugin(${PROJECT_NAME})

  yarp_install(TARGETS skinWrapper
               COMPONENT Runtime
               LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

ENDIF ()
