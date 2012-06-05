# Copyright (C) 2008 RobotCub Consortium
# Author: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PROJECT(iCubInterface)

FILE(GLOB folder_header *.h)
FILE(GLOB tmp ./canIdDiscoverer/*.h)
SET(folder_header ${folder_header} ${tmp})

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

set(BASE $ENV{ICUB_ROOT}/firmware/emBODY/)
include(../../../libraries/icubmod/embObjLib/embObjLib.conf)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../tools/canLoader/canLoaderLib
					${CMAKE_CURRENT_SOURCE_DIR}/../
                    ${iCubDev_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

if(ICUB_USE_INTERFACEGUI)
	add_definitions(-D_USE_INTERFACEGUI)
	include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../libraries/iCubInterfaceGuiLib)
endif(ICUB_USE_INTERFACEGUI)

add_library(IICcore 	RobotInterfaceRemap.cpp 
						RobotInterface.cpp 
						SkinWrapper.cpp 
						FeatureInterface.cpp 
						ControlBoardWrapper.cpp 
						ControlBoardWrapper2.cpp)
					

TARGET_LINK_LIBRARIES(IICcore canLoaderLib)                            
icub_export_library(IICcore)

