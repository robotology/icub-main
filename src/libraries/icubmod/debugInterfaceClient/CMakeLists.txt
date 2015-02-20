# Copyright: (C) 2010 RobotCub Consortium
# Authors: Marco Randazzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_device(debugInterfaceClient TYPE yarp::dev::DebugInterfaceClient INCLUDE debugInterfaceClient.h)

if(NOT SKIP_debugInterfaceClient)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}
			    ${iCubDev_INCLUDE_DIRS}
                           ${YARP_INCLUDE_DIRS})

    yarp_add_plugin(debugInterfaceClient debugInterfaceClient.h debugInterfaceClient.cpp)
    target_link_libraries(debugInterfaceClient ${YARP_LIBRARIES})

    icub_export_plugin(debugInterfaceClient)
    yarp_install(FILES debugInterfaceClient.ini  DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

endif(NOT SKIP_debugInterfaceClient)
