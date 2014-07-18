# Copyright: (C) 2012 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Name of the project must correspond to the class name that needs to be 
# instantiated by the factory - so embObjLib is not ok
SET(PROJECTNAME ethResources)
PROJECT(${PROJECTNAME})




# defined in porting.h  -- update: now seems that the porting file doen not exists anymore....

# marco.accame: to activate new mode of eoprot configuration: 
# 1. define USE_EOPROT_XML and EOPROT_CFG_OVERRIDE_CALLBACKS_IN_RUNTIME 
# 2. remove eOprot_b0x.c from compilation
# you can do taht by setting ICUB_EOPROT_DEFINED_BY_XML ON (or OFF to deactivate and stay w/ old mode)

set(ICUB_EOPROT_DEFINED_BY_XML ON)

if(ICUB_EOPROT_DEFINED_BY_XML)

add_definitions(-D_ICUB_CALLBACK_ -DUSE_EOPROT_XML -DEOPROT_CFG_OVERRIDE_CALLBACKS_IN_RUNTIME)

else(ICUB_EOPROT_DEFINED_BY_XML)

add_definitions( -D_ICUB_CALLBACK_)

endif(ICUB_EOPROT_DEFINED_BY_XML)


if(ICUB_COMPILE_EMBOBJ_LIBRARY)

if(NOT ICUB_HAS_icub_firmware_shared)
  message(ERROR " embObj: cannot find icub_firmware_shared library, turn off device embObjDevices
  embObj library can now be found in the icub-firmware-shared package that
  can be downloaded from here:
  .
       https://github.com/robotology/icub-firmware-shared
  .    ")
else(NOT ICUB_HAS_icub_firmware_shared)

#--------------------------------------------------------#
# download embObjlib from firmware repository
include(updateSVNrepository.cmake)
#--------------------------------------------------------#

#--------------------------------------------------------#
# using embObjLib.conf to define PATH VARIABLES
include(embObjLib.cmake)
#--------------------------------------------------------#

include_directories (  ${PATH_TO_CALLBACK}/embObjAnalog/usrcbk/
                       ../motionControlLib)

# Debug messages
#  message(INFO " lib_cmake - embObj_includes: ${embObj_includes}")
    
# get all files needed, collected in different lists for better organization and handling

# Common, architecture independent


set(ICUB_INTERFACE_SOURCE   ${EXTRA_FOLDER}/FeatureInterface.cpp)

if(ICUB_EOPROT_DEFINED_BY_XML)

set(NVS_EB_SOURCE           
                            )

else(ICUB_EOPROT_DEFINED_BY_XML)

set(NVS_EB_SOURCE           ${NVS_EB_SRC_FOLDER}eOprot_b01.c  
                            ${NVS_EB_SRC_FOLDER}eOprot_b02.c
                            ${NVS_EB_SRC_FOLDER}eOprot_b03.c  
                            ${NVS_EB_SRC_FOLDER}eOprot_b04.c
                            ${NVS_EB_SRC_FOLDER}eOprot_b05.c
                            ${NVS_EB_SRC_FOLDER}eOprot_b06.c  
                            ${NVS_EB_SRC_FOLDER}eOprot_b07.c  
                            ${NVS_EB_SRC_FOLDER}eOprot_b08.c
                            ${NVS_EB_SRC_FOLDER}eOprot_b09.c  
                            ${NVS_EB_SRC_FOLDER}eOprot_b10.c  
                            ${NVS_EB_SRC_FOLDER}eOprot_b11.c  
                            )

endif(ICUB_EOPROT_DEFINED_BY_XML)

set(NVS_SOURCE              ${NVS_MC_SRC_FOLDER}/EoProtocol.c
                            ${NVS_MC_SRC_FOLDER}/EoProtocolEPs.c
                            )
 
    
set(NVS_MC_SOURCE           ${NVS_MC_SRC_FOLDER}EoProtocolMC_fun.c
                            ${NVS_MC_SRC_FOLDER}EoProtocolMC_rom.c
                            )

set(NVS_MNGMNT_SOURCE       ${NVS_MNGMNT_SRC_FOLDER}EoProtocolMN_fun.c
                            ${NVS_MNGMNT_SRC_FOLDER}EoProtocolMN_rom.c
                            )

set(NVS_SENSORS_SOURCE      ${NVS_SENSORS_SRC_FOLDER}EoProtocolAS_fun.c
                            ${NVS_SENSORS_SRC_FOLDER}EoProtocolAS_rom.c
                            )

#SET_SOURCE_FILES_PROPERTIES( ${NVS_SKIN_CBK_FOLDER}eOcfg_nvsEP_sk_usrcbk_ebx.c PROPERTIES LANGUAGE CXX )

set(NVS_SKIN_SOURCE         ${NVS_SKIN_SRC_FOLDER}EoProtocolSK_fun.c
                            ${NVS_SKIN_SRC_FOLDER}EoProtocolSK_rom.c
                            )



set(NVS_CBK_SOURCE          ${NVS_MNGMNT_CBK_FOLDER}embObjLib_mngmnt_callback.c
                            ${NVS_MC_CBK_FOLDER}embObjMotionControl_callback.c
                            ${NVS_SENSORS_CBK_FOLDER}embObjAnalogSensor_callback.c
                            ${NVS_SKIN_CBK_FOLDER}/eOcfg_nvsEP_sk_usrcbk_ebx.c 
                            )


# EmbObj-src
set(CORE_FOLDER_SOURCE      ${CORE_FOLDER}/EOarray.c
                            ${CORE_FOLDER}/EoCommon.c
                            ${CORE_FOLDER}/EOconstarray.c
                            ${CORE_FOLDER}/EOconstvector.c
                            ${CORE_FOLDER}/EOdeque.c
                            ${CORE_FOLDER}/EOfifo.c
                            ${CORE_FOLDER}/EOfifoByte.c
                            ${CORE_FOLDER}/EOfifoWord.c
                            ${CORE_FOLDER}/EOlist.c
                            ${CORE_FOLDER}/EOtheErrorManager.c
                            ${CORE_FOLDER}/EOtheMemoryPool.c
                            ${CORE_FOLDER}/EOvector.c
                            ${CORE_FOLDER}/EOpacket.c
                            ${CORE_YEE}/EOYtheSystem.c
                            )

set(CORE_VEE_SOURCE         ${CORE_FOLDER}/EOVmutex.c
                            ${CORE_FOLDER}/EOVtask.c
                            ${CORE_FOLDER}/EOVtheSystem.c
                            )
                        
set(CORE_API_HEADER         ${CORE_FOLDER}/EOarray.h
                            ${CORE_FOLDER}/EoCommon.h
                            ${CORE_FOLDER}/EOconstarray.h
                            ${CORE_FOLDER}/EOconstvector.h
                            ${CORE_FOLDER}/EOdeque.h
                            ${CORE_FOLDER}/EOfifo.h
                            ${CORE_FOLDER}/EOfifoByte.h
                            ${CORE_FOLDER}/EOfifoWord.h
                            ${CORE_FOLDER}/EOlist.h
                            ${CORE_FOLDER}/EOtheErrorManager.h
                            ${CORE_FOLDER}/EOtheMemoryPool.h
                            ${CORE_FOLDER}/EOvector.h
                            ${CORE_YEE}/EOYtheSystem.h
                            ${CORE_FOLDER}/EOpacket.h
                            )
                        
set(CORE_FOLDER_HEADER      ${CORE_FOLDER}/EOarray_hid.h
                            ${CORE_FOLDER}/EoCommon_hid.h
                            ${CORE_FOLDER}/EOconstarray_hid.h
                            ${CORE_FOLDER}/EOconstvector_hid.h
                            ${CORE_FOLDER}/EOdeque_hid.h
                            ${CORE_FOLDER}/EOfifo_hid.h
                            ${CORE_FOLDER}/EOfifoByte_hid.h
                            ${CORE_FOLDER}/EOfifoWord_hid.h
                            ${CORE_FOLDER}/EOlist_hid.h
                            ${CORE_FOLDER}/EOtheErrorManager_hid.h
                            ${CORE_FOLDER}/EOtheMemoryPool_hid.h
                            ${CORE_API}/EOvector_hid.h
                            ${CORE_FOLDER}/EOpacket_hid.h
                            ${CORE_YEE}/EOYtheSystem_hid.h
                            )

# EmbObj-NV & Protocol files
set(PROT_SOURCE
                            ${PROT_FOLDER}/transport/EOnv.c
                            ${PROT_FOLDER}/transport/EOnvSet.c
                            ${PROT_FOLDER}/transport/EOreceiver.c
                            ${PROT_FOLDER}/transport/EOrop.c
                            ${PROT_FOLDER}/transport/EOropframe.c
                            ${PROT_FOLDER}/transport/EOagent.c
                            ${PROT_FOLDER}/transport/EOtheFormer.c
                            ${PROT_FOLDER}/transport/EOtheParser.c
                            ${PROT_FOLDER}/transport/EOtransceiver.c
                            ${PROT_FOLDER}/transport/EOtransmitter.c
                            ${PROT_FOLDER}/transport/EOdeviceTransceiver.c
                            ${PROT_FOLDER}/transport/EOhostTransceiver.c
                            ${PROT_FOLDER}/transport/EOconfirmationManager.c
                            ${PROT_FOLDER}/transport/EOproxy.c
                            ${PROT_FOLDER}/transport/EOprotocolConfigurator.c
                            ${PROT_FOLDER}/transport/EOnvsetDEVbuilder.c
                            ${NVS_MNGMNT_CBK_FOLDER}/protocol_callback.c
                            )

set(PROT_HEADER             ${PROT_FOLDER}/transport/EOhostTransceiver_hid.h
                            ${PROT_FOLDER}/transport/EOnv_hid.h
                            ${PROT_FOLDER}/transport/EOnvSet_hid.h
                            ${PROT_FOLDER}/transport/EOreceiver_hid.h
                            ${PROT_FOLDER}/transport/EOrop_hid.h
                            ${PROT_FOLDER}/transport/EOropframe_hid.h
                            ${PROT_FOLDER}/transport/EOtheFormer_hid.h
                            ${PROT_FOLDER}/transport/EOtheParser_hid.h
                            ${PROT_FOLDER}/transport/EOtransceiver_hid.h
                            ${PROT_FOLDER}/transport/EOtransmitter_hid.h
                            ${PROT_FOLDER}/transport/EOtransmitter_hid.h
                            ${PROT_FOLDER}/transport/EOprotocolConfigurator_hid.h
                            )


set(EXTRA_SOURCE            ${EXTRA_FOLDER}/hostTransceiver.cpp
                            ${EXTRA_FOLDER}/ethManager.cpp
                            ${EXTRA_FOLDER}/ethResource.cpp
                            ${EXTRA_FOLDER}/eoRequestsQueue.cpp)

set(DEBUG_SOURCE            ${DEBUG_FOLDER}/eODeb_eoProtoParser.c
                            ${DEBUG_FOLDER}/eOtheEthLowLevelParser.c)


set(embobj_source       
                            ${CORE_FOLDER_SOURCE} 
                            ${CORE_VEE_SOURCE}  
                            ${PROT_SOURCE}  
                            ${EXTRA_SOURCE}
                            ${PROT_FOLDER}
                            ${NVS_EB_SOURCE} 
                            ${NVS_SOURCE}
                            ${NVS_MC_SOURCE} 
                            ${NVS_MNGMNT_SOURCE} 
                            ${NVS_SENSORS_SOURCE} 
                            ${NVS_SKIN_SOURCE} 
                            ${NVS_USR_SOURCE} 
                            ${NVS_CBK_SOURCE}
                            ${ICUB_INTERFACE_SOURCE}
                            )

set(embobj_header           ${BODY_COMMON_SOURCE} 
                            ${CORE_API_HEADER} 
                            ${PROT_HEADER} 
                            ${NVS_EB_SRC_FOLDER}
                            )



SOURCE_GROUP("EmbObj Source Files" FILES ${embobj_source})
SOURCE_GROUP("EmbObj Header Files" FILES ${embobj_header})
  
include_directories(   
                            ${CMAKE_CURRENT_SOURCE_DIR}
                            ${embObj_includes}
                            ${iCubDev_INCLUDE_DIRS}
                            ${YARP_INCLUDE_DIRS}
                            )
  
add_library(${PROJECTNAME} ${embobj_source} ${embobj_header})

#add_dependencies(${PROJECTNAME} eBcode-download)
TARGET_LINK_LIBRARIES(${PROJECTNAME} debugStream ${YARP_LIBRARIES} ${ACE_LIBRARIES} )  

icub_export_library(${PROJECTNAME})

endif(NOT ICUB_HAS_icub_firmware_shared)

endif(ICUB_COMPILE_EMBOBJ_LIBRARY)

