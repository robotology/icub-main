# Copyright: (C) 2010 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PREPARE_DEVICE(embObjMotionControl TYPE embObjMotionControl
    INCLUDE embObjMotionControl.h
    WRAPPER controlboard)

 IF (NOT SKIP_embObjMotionControl)
	include(../embObjLib/embObjLib.conf)
    #  	message(INFO " embMotCtrl - embObj_includes: ${embObj_includes}")
	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS} ${iCubDev_INCLUDE_DIRS} )

#    ADD_LIBRARY(embObjMotionControl embObjMotionControl.cpp embObjMotionControl.h usrcbk/eOcfg_nvsEP_mc_usrcbk_pippo.c )
	ADD_LIBRARY(embObjMotionControl embObjMotionControl.cpp embObjMotionControl.h)
	TARGET_LINK_LIBRARIES(embObjMotionControl debugStream ethResources ${YARP_LIBRARIES} ${ACE_LIBRARIES})
	icub_export_library(embObjMotionControl)
ENDIF (NOT SKIP_embObjMotionControl)

