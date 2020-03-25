# Copyright: (C) 2012 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)

# Name of the project must correspond to the class name that needs to be 
# instantiated by the factory - so embObjLib is not ok
SET(PROJECTNAME boardTransceiver)
PROJECT(${PROJECTNAME})


option(board_tranceiver "Submit compile tests to cdash" FALSE)
mark_as_advanced (board_tranceiver)


if(board_tranceiver)
#include(iCubFindDependencies.cmake)
#find_package(icub_firmware_shared 0.0.1 COMPONENTS canProtocolLib embobj)
#find_package(yarp)



add_definitions(-D_ICUB_CALLBACK_ -DUSE_EOPROT_XML -DEOPROT_CFG_OVERRIDE_CALLBACKS_IN_RUNTIME)


# Debug messages

    
# get all files needed, collected in different lists for better organization and handling

# Common, architecture independent


set(NVS_CBK_SOURCE          
                            board_protocol_callback.c
                            
                            
                            
                            )


set(DEMO_BOARD_SOURCE       main.cpp
                            boardTransceiver.cpp
                            FeatureInterface.cpp)

set(DEMO_BOARD_HEADER       boardTransceiver.hpp
                            FeatureInterface.h
                            FeatureInterface_hid.h
                            transceiverInterface.h)



SOURCE_GROUP("EmbObj Source Files" FILES ${DEMO_BOARD_SOURCE})
SOURCE_GROUP("EmbObj Header Files" FILES ${DEMO_BOARD_HEADER})
  
include_directories(   
                            ${CMAKE_CURRENT_SOURCE_DIR}
                            ${embObj_includes}
                            ${iCubDev_INCLUDE_DIRS}
                            ${icub_firmware_shared_embobj_INCLUDE_DIR}
                            )
  
add_executable(${PROJECTNAME}       ${DEMO_BOARD_SOURCE}
                                    ${embobj_source}
                                    #${NVS_CBK_SOURCE}
                                    ${DEMO_BOARD_HEADER})

message("&&&&&&&&&&&&&&&&&&&&&&&&&&&  BOARD  &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
message(" icub_firmware_shared_embobj_LIBRARIES is ${icub_firmware_shared_embobj_LIBRARIES}\n")
message(" icub_firmware_shared_embobj_INCLUDE_DIR is ${icub_firmware_shared_embobj_INCLUDE_DIR}\n")
message("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${icub_firmware_shared_embobj_LIBRARIES} ${YARP_LIBRARIES} ${ACE_LIBRARIES} )

endif(board_tranceiver)

