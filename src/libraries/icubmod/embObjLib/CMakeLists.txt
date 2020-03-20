# Copyright: (C) 2012 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Name of the project must correspond to the class name that needs to be
# instantiated by the factory - so embObjLib is not ok
SET(PROJECTNAME ethResources)
PROJECT(${PROJECTNAME})



add_definitions(-D_ICUB_CALLBACK_ -DUSE_EOPROT_XML -DEOPROT_CFG_OVERRIDE_CALLBACKS_IN_RUNTIME)


if(ICUB_COMPILE_EMBOBJ_LIBRARY)

if(NOT ICUB_HAS_icub_firmware_shared)
  message(ERROR " embObj: cannot find icub_firmware_shared library, turn off device embObjDevices
  embObj library can now be found in the icub-firmware-shared package that
  can be downloaded from here:
  .
       https://github.com/robotology/icub-firmware-shared
  .    ")
else(NOT ICUB_HAS_icub_firmware_shared)
    set(FIRMWARE_SHARED_INC         ${icub_firmware_shared_canProtocolLib_INCLUDE_DIR})

#--------------------------------------------------------#
# download embObjlib from firmware repository
include(updateSVNrepository.cmake)
#--------------------------------------------------------#

#--------------------------------------------------------#
# using embObjLib.conf to define PATH VARIABLES
include(embObjLib.cmake)
#--------------------------------------------------------#

include_directories (  ${PATH_TO_CALLBACK}/embObjAnalog/usrcbk/
                       ../skinLib)

# Debug messages
#  message(INFO " lib_cmake - embObj_includes: ${embObj_includes}")

# get all files needed, collected in different lists for better organization and handling

# Common, architecture independent


set(ICUB_INTERFACE_SOURCE   ${EXTRA_FOLDER}/FeatureInterface.cpp)

# TODO these files will be compiled in the icub-firmware-shared lib. They as to be removed.
#set(NVS_SOURCE              ${NVS_MC_SRC_FOLDER}/EoProtocol.c
#                            ${NVS_MC_SRC_FOLDER}/EoProtocolEPs.c
#                            ${PROT_FOLDER}/icub/EoError.c
#                            ${PROT_FOLDER}/icub/EoBoards.c
#                            ${PROT_FOLDER}/icub/EoMotionControl.c
#                            ${PROT_FOLDER}/icub/EoAnalogSensors.c
#                            ${PROT_FOLDER}/icub/EoManagement.c
#                            )
#
#
#set(NVS_MC_SOURCE           ${NVS_MC_SRC_FOLDER}EoProtocolMC_fun.c
#                            ${NVS_MC_SRC_FOLDER}EoProtocolMC_rom.c
#                            )
#
#set(NVS_MNGMNT_SOURCE       ${NVS_MNGMNT_SRC_FOLDER}EoProtocolMN_fun.c
#                            ${NVS_MNGMNT_SRC_FOLDER}EoProtocolMN_rom.c
#                            )
#
#set(NVS_SENSORS_SOURCE      ${NVS_SENSORS_SRC_FOLDER}EoProtocolAS_fun.c
#                            ${NVS_SENSORS_SRC_FOLDER}EoProtocolAS_rom.c
#                            )
#
#set(NVS_SKIN_SOURCE         ${NVS_SKIN_SRC_FOLDER}EoProtocolSK_fun.c
#                            ${NVS_SKIN_SRC_FOLDER}EoProtocolSK_rom.c
#                            )
#
#
#
#set(NVS_CBK_SOURCE          ${NVS_CBK_FOLDER}EoProtocolMN_fun_userdef.c
#                            ${NVS_CBK_FOLDER}EoProtocolMC_fun_userdef.c
#                            ${NVS_CBK_FOLDER}EoProtocolAS_fun_userdef.c
#                            ${NVS_CBK_FOLDER}EoProtocolSK_fun_userdef.c
#                            )
#
#
## EmbObj-src
#set(CORE_FOLDER_SOURCE      ${CORE_FOLDER}/EOarray.c
#                            ${CORE_FOLDER}/EoCommon.c
#                            ${CORE_FOLDER}/EOconstarray.c
#                            ${CORE_FOLDER}/EOconstvector.c
#                            ${CORE_FOLDER}/EOdeque.c
#                            ${CORE_FOLDER}/EOfifo.c
#                            ${CORE_FOLDER}/EOfifoByte.c
#                            ${CORE_FOLDER}/EOfifoWord.c
#                            ${CORE_FOLDER}/EOlist.c
#                            ${CORE_FOLDER}/EOtheErrorManager.c
#                            ${CORE_FOLDER}/EOtheMemoryPool.c
#                            ${CORE_FOLDER}/EOvector.c
#                            ${CORE_FOLDER}/EOpacket.c
#                            ${CORE_YEE}/EOYtheSystem.c
#                            ${CORE_YEE}/EOYmutex.c
#                            )
#
#set(CORE_VEE_SOURCE         ${CORE_FOLDER}/EOVmutex.c
#                            ${CORE_FOLDER}/EOVtask.c
#                            ${CORE_FOLDER}/EOVtheSystem.c
#                            )
#
#set(CORE_API_HEADER         ${CORE_FOLDER}/EOarray.h
#                            ${CORE_FOLDER}/EoCommon.h
#                            ${CORE_FOLDER}/EOconstarray.h
#                            ${CORE_FOLDER}/EOconstvector.h
#                            ${CORE_FOLDER}/EOdeque.h
#                            ${CORE_FOLDER}/EOfifo.h
#                            ${CORE_FOLDER}/EOfifoByte.h
#                            ${CORE_FOLDER}/EOfifoWord.h
#                            ${CORE_FOLDER}/EOlist.h
#                            ${CORE_FOLDER}/EOtheErrorManager.h
#                            ${CORE_FOLDER}/EOtheMemoryPool.h
#                            ${CORE_FOLDER}/EOvector.h
#                            ${CORE_YEE}/EOYtheSystem.h
#                            ${CORE_FOLDER}/EOpacket.h
#                            )
#
#set(CORE_FOLDER_HEADER      ${CORE_FOLDER}/EOarray_hid.h
#                            ${CORE_FOLDER}/EoCommon_hid.h
#                            ${CORE_FOLDER}/EOconstarray_hid.h
#                            ${CORE_FOLDER}/EOconstvector_hid.h
#                            ${CORE_FOLDER}/EOdeque_hid.h
#                            ${CORE_FOLDER}/EOfifo_hid.h
#                            ${CORE_FOLDER}/EOfifoByte_hid.h
#                            ${CORE_FOLDER}/EOfifoWord_hid.h
#                            ${CORE_FOLDER}/EOlist_hid.h
#                            ${CORE_FOLDER}/EOtheErrorManager_hid.h
#                            ${CORE_FOLDER}/EOtheMemoryPool_hid.h
#                            ${CORE_API}/EOvector_hid.h
#                            ${CORE_FOLDER}/EOpacket_hid.h
#                            ${CORE_YEE}/EOYtheSystem_hid.h
#                            )
#
## EmbObj-NV & Protocol files
#set(PROT_SOURCE
#                            ${PROT_FOLDER}/transport/EOnv.c
#                            ${PROT_FOLDER}/transport/EOnvSet.c
#                            ${PROT_FOLDER}/transport/EOreceiver.c
#                            ${PROT_FOLDER}/transport/EOrop.c
#                            ${PROT_FOLDER}/transport/EOropframe.c
#                            ${PROT_FOLDER}/transport/EOagent.c
#                            ${PROT_FOLDER}/transport/EOtheFormer.c
#                            ${PROT_FOLDER}/transport/EOtheParser.c
#                            ${PROT_FOLDER}/transport/EOtransceiver.c
#                            ${PROT_FOLDER}/transport/EOtransmitter.c
#                            ${PROT_FOLDER}/transport/EOhostTransceiver.c
#                            ${PROT_FOLDER}/transport/EOconfirmationManager.c
#                            ${PROT_FOLDER}/transport/EOproxy.c
#                            )
#
#set(PROT_HEADER             ${PROT_FOLDER}/transport/EOhostTransceiver_hid.h
#                            ${PROT_FOLDER}/transport/EOnv_hid.h
#                            ${PROT_FOLDER}/transport/EOnvSet_hid.h
#                            ${PROT_FOLDER}/transport/EOreceiver_hid.h
#                            ${PROT_FOLDER}/transport/EOrop_hid.h
#                            ${PROT_FOLDER}/transport/EOropframe_hid.h
#                            ${PROT_FOLDER}/transport/EOtheFormer_hid.h
#                            ${PROT_FOLDER}/transport/EOtheParser_hid.h
#                            ${PROT_FOLDER}/transport/EOtransceiver_hid.h
#                            ${PROT_FOLDER}/transport/EOtransmitter_hid.h
#                            ${PROT_FOLDER}/transport/EOtransmitter_hid.h
#                            )


set(EXTRA_SOURCE            ${EXTRA_FOLDER}/hostTransceiver.cpp
                            ${EXTRA_FOLDER}/ethManager.cpp
                            ${EXTRA_FOLDER}/ethResource.cpp
                            ${EXTRA_FOLDER}/ethMonitorPresence.cpp
                            ${EXTRA_FOLDER}/ethBoards.cpp
                            ${EXTRA_FOLDER}/ethSender.cpp
                            ${EXTRA_FOLDER}/ethReceiver.cpp
                            ${EXTRA_FOLDER}/ethParser.cpp
                            ${EXTRA_FOLDER}/IethResource.cpp
                            ${EXTRA_FOLDER}/fakeEthResource.cpp
                            ${EXTRA_FOLDER}/serviceParser.cpp
                            ${EXTRA_FOLDER}/theNVmanager.cpp
                            ${EXTRA_FOLDER}/embObjGeneralDevPrivData.cpp
                            )
                            
message("EXTRA_SOURCE= " ${EXTRA_SOURCE})

set(DEBUG_SOURCE            ${DEBUG_FOLDER}/eODeb_eoProtoParser.c
                            ${DEBUG_FOLDER}/eOtheEthLowLevelParser.c)

option(NETWORK_PERFORMANCE_BENCHMARK "Enable embedded network perfomance bamchmark." OFF)
mark_as_advanced(NETWORK_PERFORMANCE_BENCHMARK)

if(NETWORK_PERFORMANCE_BENCHMARK)

set(TOOLS_FOLDER    ${EXTRA_FOLDER}/tools)
set(TOOLS_HEADER    ${TOOLS_FOLDER}/include/PeriodicEventsVerifier.h)
set(TOOLS_SOURCE    ${TOOLS_FOLDER}/src/PeriodicEventsVerifier.cpp 
                    ${TOOLS_FOLDER}/src/embot_tools.cpp)
else()

set(TOOLS_FOLDER  ""  )
set(TOOLS_HEADER  ""  )
set(TOOLS_SOURCE  ""  )

endif()


message("tools source= " ${TOOLS_SOURCE})

set(embobj_source
#                            ${CORE_FOLDER_SOURCE}
#                            ${CORE_VEE_SOURCE}
#                            ${PROT_SOURCE}
                            ${EXTRA_SOURCE}
#                            ${PROT_FOLDER}
#                            ${NVS_SOURCE}
#                            ${NVS_MC_SOURCE}
#                            ${NVS_MNGMNT_SOURCE}
#                            ${NVS_SENSORS_SOURCE}
#                            ${NVS_SKIN_SOURCE}
#                            ${NVS_USR_SOURCE}
#                            ${NVS_CBK_SOURCE}
                            ${ICUB_INTERFACE_SOURCE}
                            ${TOOLS_SOURCE} )

set(embobj_header           # ${BODY_COMMON_SOURCE} # TODO check this var, it is defined here:src/tools/embObjProtoTools/nvListPrinter/CMakeLists.txt probably it is not set(yes in my case
                            # ${CORE_API_HEADER}
                            # ${PROT_HEADER}
                            # ${NVS_EB_SRC_FOLDER} #TODO it is robotconfig/v1/eprotocol/, should be exported by icub-firmware-shared.
                            ${TOOLS_HEADER} )

SOURCE_GROUP("EmbObj Source Files" FILES ${embobj_source})
SOURCE_GROUP("EmbObj Header Files" FILES ${embobj_header})

# TODO this will be removed, target_link_library will be sufficient
# if linking the new cmake targets
include_directories(SYSTEM ${ACE_INCLUDE_DIRS} ${FIRMWARE_SHARED_INC})

# TODO this will be substituted by target_include_directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${embObj_includes}
                    # ${iCubDev_INCLUDE_DIRS} #TODO It should be not used (check where is used and why)
                    ${CMAKE_CURRENT_SOURCE_DIR}/../motionControlLib) #TODO not beautiful but ok-ish

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
add_library(${PROJECTNAME} SHARED ${embobj_source} ${embobj_header})
    
install(TARGETS ${PROJECTNAME} DESTINATION lib)

#add_dependencies(${PROJECTNAME} eBcode-download)
# TODO replace the *_LIBRARIES with the used cmake targets, there will be also the icub-firmware-shared lib.
TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES} ${ACE_LIBRARIES} )

icub_export_library(${PROJECTNAME})

endif(NOT ICUB_HAS_icub_firmware_shared)

endif(ICUB_COMPILE_EMBOBJ_LIBRARY)

