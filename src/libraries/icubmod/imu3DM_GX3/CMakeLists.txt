#
# Copyright (C) 2013 RobotCub Consortium
# Author: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

SET(PROJECTNAME imu3DM_GX3)
PROJECT(${PROJECTNAME})

yarp_prepare_plugin(${PROJECTNAME} CATEGORY device TYPE yarp::dev::imu3DM_GX3 INCLUDE 3dm_gx3.h EXTRA_CONFIG WRAPPER=inertial)

if (ENABLE_${PROJECTNAME})
    INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
    yarp_add_plugin(${PROJECTNAME} 3dm_gx3.cpp 3dm_gx3.h )
    target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})
    icub_export_plugin(${PROJECTNAME})

  yarp_install(TARGETS ${PROJECTNAME}
               COMPONENT Runtime
               LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

endif (ENABLE_${PROJECTNAME})
