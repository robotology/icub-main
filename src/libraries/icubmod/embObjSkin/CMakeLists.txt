# Copyright: (C) 2010 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PREPARE_DEVICE(embObjSkin TYPE EmbObjSkin INCLUDE embObjSkin.h WRAPPER grabber)

if (NOT SKIP_embObjSkin)
    include(../embObjLib/embObjLib.conf)
#  	message(INFO " embObjSkin - embObj_includes: ${embObj_includes}")
	include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                       ${YARP_INCLUDE_DIRS}
                       ${iCubDev_INCLUDE_DIRS})

	add_library(embObjSkin embObjSkin.h embObjSkin.cpp)
	target_link_libraries(embObjSkin ethResources ${YARP_LIBRARIES})
#	TARGET_LINK_LIBRARIES(embObjSkin DebugStream ethResources ${YARP_LIBRARIES} ${ACE_LIBRARIES})
	icub_export_library(embObjSkin)
endif (NOT SKIP_embObjSkin)
