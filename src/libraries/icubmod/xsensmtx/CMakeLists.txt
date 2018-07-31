#
# Copyright (C) 2008 RobotCub Consortium
# Author: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

yarp_prepare_plugin(xsensmtx CATEGORY device TYPE yarp::dev::XSensMTx INCLUDE XSensMTx.h EXTRA_CONFIG WRAPPER=inertial)

if (NOT SKIP_xsensmtx)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
    yarp_add_plugin(xsensmtx MTComm.cpp  MTComm.h  XSensMTx.cpp  XSensMTx.h)
    target_link_libraries(xsensmtx ${YARP_LIBRARIES})
    icub_export_plugin(xsensmtx)
    yarp_install(FILES xsensmtx.ini  DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})
endif (NOT SKIP_xsensmtx)
