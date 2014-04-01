# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PREPARE_DEVICE(socketcan TYPE SocketCan
    INCLUDE SocketCan.h 
    WRAPPER controlboard)

IF (NOT SKIP_socketcan)
 
    if (WIN32)
		MESSAGE("socketcan: sorry not available in windows. Turn off the device.") 
    ELSE(WIN32) 
	    INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS})
	    yarp_add_plugin(socketcan SocketCan.cpp SocketCan.h)
	    TARGET_LINK_LIBRARIES(socketcan ${YARP_LIBRARIES})   
	    icub_export_library(socketcan)
    ENDIF(WIN32)

ENDIF (NOT SKIP_socketcan)
