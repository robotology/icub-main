# Copyright: (C) 2010 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PREPARE_DEVICE(ethResources TYPE ethResources
    INCLUDE ethManager.h 
    WRAPPER "")

 IF (NOT SKIP_ethResources)
	set(BASE $ENV{ICUB_ROOT}/firmware/emBODY/)
    include(../embObjLib/embObjLib.conf)
  	message(INFO " embMotCtrl - embObj_includes: ${embObj_includes}")
    INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS} ${iCubDev_INCLUDE_DIRS} )

    ADD_LIBRARY(ethResources ethManager.cpp ethManager.h)
    TARGET_LINK_LIBRARIES(ethResources pthread hostTransceiver ${YARP_LIBRARIES} ${ACE_LIBRARIES})
    icub_export_library(ethResources)
ENDIF (NOT SKIP_ethResources)

