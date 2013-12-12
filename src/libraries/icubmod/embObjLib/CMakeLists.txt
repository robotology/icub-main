# Copyright: (C) 2012 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Name of the project must correspond to the class name that needs to be 
# instantiated by the factory - so embObjLib is not ok
SET(PROJECTNAME ethResources)
PROJECT(${PROJECTNAME})



# defined in porting.h  -- update: now seems that the porting file doen not exists anymore....
add_definitions( -D_ICUB_CALLBACK_)

PREPARE_DEVICE(${PROJECTNAME} TYPE ${PROJECTNAME}  INCLUDE ethManager.h ethResource.h)

if(ICUB_COMPILE_EMBOBJ_LIBRARY)
#--------------------------------------------------------#
# download embObjlib from firmware repository
include(updateSVNrepository.cmake)
#--------------------------------------------------------#

#--------------------------------------------------------#
# using embObjLib.conf to define PATH VARIABLES
include(embObjLib.cmake)
#--------------------------------------------------------#

include_directories (  ${PATH_TO_CALLBACK}/embObjAnalog/usrcbk/)

# Debug messages
#  message(INFO " lib_cmake - embObj_includes: ${embObj_includes}")
  	
# get all files needed, collected in different lists for better organization and handling

# Common, architecture independent

set(ICUB_INTERFACE_SOURCE	${EXTRA_FOLDER}/FeatureInterface.cpp)

# Network Variable configuration# Network Variable configuration
set(NVS_EB_SOURCE 			${NVS_EB_SRC_FOLDER}eOcfg_EPs_board.c  
							${NVS_EB_SRC_FOLDER}eOcfg_EPs_eb1.c
							${NVS_EB_SRC_FOLDER}eOcfg_EPs_eb2.c
							${NVS_EB_SRC_FOLDER}eOcfg_EPs_eb3.c  
							${NVS_EB_SRC_FOLDER}eOcfg_EPs_eb4.c
							${NVS_EB_SRC_FOLDER}eOcfg_EPs_eb5.c
							${NVS_EB_SRC_FOLDER}eOcfg_EPs_eb6.c  
							${NVS_EB_SRC_FOLDER}eOcfg_EPs_eb7.c  
							${NVS_EB_SRC_FOLDER}eOcfg_EPs_eb8.c
							${NVS_EB_SRC_FOLDER}eOcfg_EPs_eb9.c  )

	
set(NVS_MC_SOURCE 			${NVS_MC_SRC_FOLDER}eOcfg_nvsEP_mc.c
							${NVS_MC_SRC_FOLDER}eOcfg_nvsEP_mc_any_con_bodypart.c
							${NVS_MC_SRC_FOLDER}eOcfg_nvsEP_mc_any_con_jxx.c
							${NVS_MC_SRC_FOLDER}eOcfg_nvsEP_mc_any_con_jxxdefault.c
							${NVS_MC_SRC_FOLDER}eOcfg_nvsEP_mc_any_con_mxx.c
							${NVS_MC_SRC_FOLDER}eOcfg_nvsEP_mc_any_con_mxxdefault.c
							${NVS_MC_SRC_FOLDER}eOcfg_nvsEP_mc_any_con_cxx.c
							${NVS_MC_SRC_FOLDER}eOcfg_nvsEP_mc_any_con_cxxdefault.c
							
							${NVS_MC_SRC_FOLDER}eOcfg_nvsEP_mc_lowerarm_con.c
							${NVS_MC_SRC_FOLDER}eOcfg_nvsEP_mc_lowerarm_usr.c

							${NVS_MC_SRC_FOLDER}eOcfg_nvsEP_mc_lowerleg_con.c
							${NVS_MC_SRC_FOLDER}eOcfg_nvsEP_mc_lowerleg_usr.c

							${NVS_MC_SRC_FOLDER}eOcfg_nvsEP_mc_torso_con.c
							${NVS_MC_SRC_FOLDER}eOcfg_nvsEP_mc_torso_usr.c

							${NVS_MC_SRC_FOLDER}eOcfg_nvsEP_mc_upperarm_con.c
							${NVS_MC_SRC_FOLDER}eOcfg_nvsEP_mc_upperarm_usr.c

							${NVS_MC_SRC_FOLDER}eOcfg_nvsEP_mc_upperleg_con.c
							${NVS_MC_SRC_FOLDER}eOcfg_nvsEP_mc_upperleg_usr.c 
							)						

set(NVS_MNGMNT_SOURCE		${NVS_MNGMNT_SRC_FOLDER}eOcfg_nvsEP_mn.c
							${NVS_MNGMNT_SRC_FOLDER}eOcfg_nvsEP_mn_appl_con.c
							${NVS_MNGMNT_SRC_FOLDER}eOcfg_nvsEP_mn_appl_usr.c
							${NVS_MNGMNT_SRC_FOLDER}eOcfg_nvsEP_mn_comm_con.c
							${NVS_MNGMNT_SRC_FOLDER}eOcfg_nvsEP_mn_comm_usr.c )

set(NVS_SENSORS_SOURCE		${NVS_SENSORS_SRC_FOLDER}eOcfg_nvsEP_as.c
							${NVS_SENSORS_SRC_FOLDER}eOcfg_nvsEP_as_any_con_body.c  
							${NVS_SENSORS_SRC_FOLDER}eOcfg_nvsEP_as_any_con_mxx.c 
							${NVS_SENSORS_SRC_FOLDER}eOcfg_nvsEP_as_any_con_mxxdefault.c 
							${NVS_SENSORS_SRC_FOLDER}eOcfg_nvsEP_as_any_con_sxx.c 
							${NVS_SENSORS_SRC_FOLDER}eOcfg_nvsEP_as_any_con_sxxdefault.c 
							${NVS_SENSORS_SRC_FOLDER}eOcfg_nvsEP_as_onemais_con.c 
							${NVS_SENSORS_SRC_FOLDER}eOcfg_nvsEP_as_onemais_usr.c 
							${NVS_SENSORS_SRC_FOLDER}eOcfg_nvsEP_as_onestrain_con.c 
							${NVS_SENSORS_SRC_FOLDER}eOcfg_nvsEP_as_onestrain_usr.c 
							)

#SET_SOURCE_FILES_PROPERTIES( ${NVS_SKIN_CBK_FOLDER}eOcfg_nvsEP_sk_usrcbk_ebx.c PROPERTIES LANGUAGE CXX )

set(NVS_SKIN_SOURCE			${NVS_SKIN_SRC_FOLDER}eOcfg_nvsEP_sk.c
							${NVS_SKIN_CBK_FOLDER}eOcfg_nvsEP_sk_usrcbk_ebx.c
							${NVS_SKIN_SRC_FOLDER}eOcfg_nvsEP_sk_emsboard_con.c
							${NVS_SKIN_SRC_FOLDER}eOcfg_nvsEP_sk_emsboard_usr.c )



set(NVS_CBK_SOURCE      ${NVS_MNGMNT_CBK_FOLDER}embObjLib_mngmnt_callback.c
                        ${NVS_MC_CBK_FOLDER}embObjMotionControl_callback.c
                        ${NVS_SENSORS_CBK_FOLDER}embObjAnalogSensor_callback.c        )


# EmbObj-src
set(CORE_FOLDER_SOURCE  	${CORE_FOLDER}/EOarray.c
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

set(CORE_VEE_SOURCE			${CORE_FOLDER}/EOVmutex.c
							${CORE_FOLDER}/EOVtask.c
							${CORE_FOLDER}/EOVtheSystem.c
							)
						
set(CORE_API_HEADER	  			${CORE_FOLDER}/EOarray.h
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
#						${CORE_FOLDER}/EOVmutex.h
#						${CORE_FOLDER}/EOVstorage.h
#						${CORE_FOLDER}/EOVtask.h
#						${CORE_FOLDER}/EOVtheSystem.h
                        
						)						
						
set(CORE_FOLDER_HEADER	  			${CORE_FOLDER}/EOarray_hid.h
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
						${PROT_FOLDER}/prot/EOnv.c
						${PROT_FOLDER}/prot/EOnvsCfg.c
						${PROT_FOLDER}/prot/EOreceiver.c
						${PROT_FOLDER}/prot/EOrop.c
						${PROT_FOLDER}/prot/EOropframe.c
						${PROT_FOLDER}/prot/EOtheAgent.c
						${PROT_FOLDER}/prot/EOtheFormer.c
						${PROT_FOLDER}/prot/EOtheParser.c
						${PROT_FOLDER}/prot/EOtransceiver.c
						${PROT_FOLDER}/prot/EOtransmitter.c
						${PROT_FOLDER}/prot/EOtreenode.c
						${PROT_FOLDER}/prot/EOtheBOARDtransceiver.c
						${PROT_FOLDER}/prot/EOhostTransceiver.c
						${PROT_FOLDER}/prot/EOVstorage.c
						${PROT_FOLDER}/prot/EOmatrix3d.c
						${NVS_MNGMNT_CBK_FOLDER}/protocol_callback.c)

set(PROT_HEADER  		${PROT_FOLDER}/prot/EOhostTransceiver_hid.h
						${PROT_FOLDER}/prot/EOnv_hid.h
						${PROT_FOLDER}/prot/EOnvsCfg_hid.h
						${PROT_FOLDER}/prot/EOreceiver_hid.h
						${PROT_FOLDER}/prot/EOrop_hid.h
						${PROT_FOLDER}/prot/EOropframe_hid.h
						${PROT_FOLDER}/prot/EOtheAgent_hid.h
						${PROT_FOLDER}/prot/EOtheFormer_hid.h
						${PROT_FOLDER}/prot/EOtheParser_hid.h
						${PROT_FOLDER}/prot/EOtransceiver_hid.h
						${PROT_FOLDER}/prot/EOtransmitter_hid.h
						${PROT_FOLDER}/prot/EOtreenode_hid.h)


#set(IPNET_SOURCE		${IPNET_FOLDER}/EOpacket.c)
#set(IPNET_HEADER		${IPNET_FOLDER}/EOpacket_hid.h)

set(EXTRA_SOURCE 		${EXTRA_FOLDER}/hostTransceiver.cpp
						${EXTRA_FOLDER}/ethManager.cpp
						${EXTRA_FOLDER}/ethResource.cpp
#						${EXTRA_FOLDER}/tools/debugFunctions.cpp
						${EXTRA_FOLDER}/eoRequestsQueue.cpp)

set(DEBUG_SOURCE    ${DEBUG_FOLDER}/eODeb_eoProtoParser.c
                    ${DEBUG_FOLDER}/eOtheEthLowLevelParser.c)


set(embobj_source 		
						${CORE_FOLDER_SOURCE} 
						${CORE_VEE_SOURCE}  
						${PROT_SOURCE}  
						${EXTRA_SOURCE}
						${PROT_FOLDER}
						${NVS_EB_SOURCE} 
						${NVS_MC_SOURCE} 
						${NVS_MNGMNT_SOURCE} 
						${NVS_SENSORS_SOURCE} 
						${NVS_SKIN_SOURCE} 
						${NVS_USR_SOURCE} 
						${NVS_CBK_SOURCE}
						${ICUB_INTERFACE_SOURCE}
						${DEBUG_SOURCE})

set(embobj_header 		${BODY_COMMON_SOURCE} 
						${CORE_API_HEADER} 
						${PROT_HEADER} 
						${NVS_EB_SRC_FOLDER})
					


SOURCE_GROUP("EmbObj Source Files" FILES ${embobj_source})
SOURCE_GROUP("EmbObj Header Files" FILES ${embobj_header})
  
include_directories (   ${CMAKE_CURRENT_SOURCE_DIR}
                        ${embObj_includes}
                        ${iCubDev_INCLUDE_DIRS}
                        ${YARP_INCLUDE_DIRS})
  
add_library(${PROJECTNAME} ${embobj_source} ${embobj_header})

#add_dependencies(${PROJECTNAME} eBcode-download)
TARGET_LINK_LIBRARIES(${PROJECTNAME} debugStream ${YARP_LIBRARIES} ${ACE_LIBRARIES} )  

icub_export_library(${PROJECTNAME})

endif(ICUB_COMPILE_EMBOBJ_LIBRARY)
