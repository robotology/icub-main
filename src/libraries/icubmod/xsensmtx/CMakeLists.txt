#
# Copyright (C) 2008 RobotCub Consortium
# Author: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

yarp_prepare_plugin(xsensmtx
                    CATEGORY device
                    TYPE yarp::dev::XSensMTx
                    INCLUDE XSensMTx.h
                    EXTRA_CONFIG WRAPPER=inertial)

if (NOT SKIP_xsensmtx)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
    yarp_add_plugin(xsensmtx MTComm.cpp
                             MTComm.h
                             XSensMTx.cpp
                             XSensMTx.h)
    target_link_libraries(xsensmtx PRIVATE YARP::YARP_OS
                                           YARP::YARP_sig
                                           YARP::YARP_dev)
    yarp_install(TARGETS xsensmtx
                 EXPORT icub-targets
                 COMPONENT Runtime
                 LIBRARY DESTINATION  ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION  ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})
endif (NOT SKIP_xsensmtx)
