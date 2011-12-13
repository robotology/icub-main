# Copyright (C) 2008 RobotCub Consortium
# Author: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PROJECT(iCubInterface)

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)

FILE(GLOB tmp ./canIdDiscoverer/*.h)
SET(folder_header ${folder_header} ${tmp})

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../tools/canLoader/canLoaderLib
                    ${iCubDev_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

if(ICUB_USE_INTERFACEGUI)
	add_definitions(-D_USE_INTERFACEGUI)
	include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../libraries/iCubInterfaceGuiLib)
endif(ICUB_USE_INTERFACEGUI)

ADD_EXECUTABLE(iCubInterface ${folder_source} ${folder_header})



if(ICUB_USE_INTERFACEGUI)
    TARGET_LINK_LIBRARIES(iCubInterface iCubDev 
                                    ${YARP_LIBRARIES}
                                    canLoaderLib 
                                    icubmod
                                    iCubInterfaceGuiLib)
else()
    TARGET_LINK_LIBRARIES(iCubInterface iCubDev 
                                    ${YARP_LIBRARIES}
                                    canLoaderLib 
                                    icubmod)
endif()

INSTALL(TARGETS iCubInterface DESTINATION bin COMPONENT Core)

