#
# Copyright (C) 2015 RobotCub Consortium
# Author: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

SET(PROJECTNAME imuST_M1)

yarp_prepare_plugin(${PROJECTNAME} CATEGORY device TYPE yarp::dev::imuST_M1 INCLUDE ST_M1.h EXTRA_CONFIG WRAPPER=inertial)

if (ENABLE_${PROJECTNAME})

    if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        MESSAGE(SEND_ERROR "${PROJECTNAME}: device not available yet on OS X platform, please turn off the device")
        return()
    elseif(WIN32)
        MESSAGE(SEND_ERROR "${PROJECTNAME}: device not available yet on Windows platform, please turn off the device")
        return()
  endif()


    INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
    yarp_add_plugin(${PROJECTNAME} ST_M1.cpp ST_M1.h ST_M1_dataType.h)
    target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})
    icub_export_plugin(${PROJECTNAME})

  yarp_install(TARGETS ${PROJECTNAME}
               COMPONENT Runtime
               LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

endif (ENABLE_${PROJECTNAME})
