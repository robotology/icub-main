# Copyright (C) 2008 RobotCub Consortium
# Author: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.



project(iCubInterface)

add_definitions( -D_ICUB_CALLBACK_ )

file(GLOB folder_source ./robotInterface/*.cpp)
set(folder_source ${folder_source} main.cpp)

file(GLOB folder_header *.h)
file(GLOB tmp ./canIdDiscoverer/*.h)
file(GLOB rih ./robotInterface/*.h)
set(folder_header ${folder_header} ${tmp} ${rih} )

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

set(BASE $ENV{ICUB_ROOT}/firmware/emBODY/)
include(../../libraries/icubmod/embObjLib/embObjLib.conf)



include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../tools/canLoader/canLoaderLib
                    ${CMAKE_CURRENT_SOURCE_DIR}/robotInterface/
                    ${iCubDev_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${CMAKE_SOURCE_DIR}/src/libraries/icubmod/ControlBoardWrapper2)


if(ICUB_USE_INTERFACEGUI)
    add_definitions(-D_USE_INTERFACEGUI)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../libraries/iCubInterfaceGuiLib)
endif(ICUB_USE_INTERFACEGUI)

add_executable(iCubInterface ${folder_source} ${folder_header})

if(ICUB_USE_INTERFACEGUI)
    target_link_libraries(iCubInterface iCubDev
                                    ${YARP_LIBRARIES}
                                    canLoaderLib
                                    icubmod
                                    iCubInterfaceGuiLib
                                    DebugStream)
else()
    target_link_libraries(iCubInterface iCubDev
                                    ${YARP_LIBRARIES}
                                    canLoaderLib
                                    icubmod
                                    DebugStream)
endif()

install(TARGETS iCubInterface DESTINATION bin COMPONENT Core)


