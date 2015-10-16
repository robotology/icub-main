#
# Copyright (C) 2015 RobotCub Consortium
# Author: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

SET(PROJECTNAME imuST_M1)

yarp_prepare_device(${PROJECTNAME} TYPE yarp::dev::imuST_M1 INCLUDE ST_M1.h WRAPPER inertial)

if (ENABLE_${PROJECTNAME})

    if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        MESSAGE(SEND_ERROR "${PROJECTNAME}: device not available yet on OS X platform, please turn off the device")
        return()
    elseif(UNIX)
        find_package(Libv4l2 REQUIRED)
        find_package(Libv4lconvert REQUIRED)

        if( NOT Libv4l2_FOUND )
            MESSAGE(SEND_ERROR "${PROJECTNAME}: library libv4l2 not found, please turn off device.")
            return()
        endif(NOT Libv4l2_FOUND )

        if( NOT Libv4lconvert_FOUND )
            MESSAGE(SEND_ERROR "${PROJECTNAME}: libv4lconvert not found, please turn off device.'")
            return()
        endif(NOT Libv4lconvert_FOUND )

    elseif(WIN32)
        MESSAGE(SEND_ERROR "${PROJECTNAME}: device not available yet on Windows platform, please turn off the device")
        return()
    else()
        MESSAGE(SEND_ERROR "${PROJECTNAME}: OS not supported, please turn off the device")
        return()
  endif()


    INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS})
    yarp_add_plugin(${PROJECTNAME} ST_M1.cpp ST_M1.h ST_M1_dataType.h)
    target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})
    icub_export_plugin(${PROJECTNAME})
    yarp_install(FILES imuST_M1.ini  DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})
endif (ENABLE_${PROJECTNAME})
