# Copyright: (C) 2010 RobotCub Consortium
# Authors: Marco Randazzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PREPARE_DEVICE(debugInterfaceClient TYPE DebugInterfaceClient INCLUDE debugInterfaceClient.h WRAPPER controlboard)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
			${iCubDev_INCLUDE_DIRS}
                       ${YARP_INCLUDE_DIRS})

yarp_add_plugin(debugInterfaceClient debugInterfaceClient.h debugInterfaceClient.cpp)
target_link_libraries(debugInterfaceClient ${YARP_LIBRARIES})

icub_export_library(debugInterfaceClient)
