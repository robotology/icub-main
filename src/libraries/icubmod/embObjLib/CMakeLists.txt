# Copyright: (C) 2012 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Name of the project must correspond to the class name that needs to be 
# instantiated by the factory - so embObjLib is not ok
SET(PROJECTNAME ethResources)
PROJECT(${PROJECTNAME})

# defined in porting.h
#add_definitions( -D_ICUB_CALLBACK_ -DEOCFG_NVSEP_MC_OVERRIDE -DEOCFG_NVSEP_SK_OVERRIDE)
add_definitions( -D_ICUB_CALLBACK_)

PREPARE_DEVICE(${PROJECTNAME} TYPE ${PROJECTNAME}  INCLUDE ethManager.h )

set(EO_BASE_DIR         ${CMAKE_SOURCE_DIR}/../firmware/emBODY/)

#set(BODY_COMMON_FOLDER  ${EO_BASE_DIR}any-arch/body/embenv/envcom/)
#set(CORE_FOLDER     ${EO_BASE_DIR}any-arch/sys/embobj/core/src/)
#set(CORE_API        ${EO_BASE_DIR}any-arch/sys/embobj/core/api/)
#set(CORE_YEE          ${EO_BASE_DIR}any-arch/sys/embobj/core-yee/)
#set(CORE_VEE_FOLDER   ${EO_BASE_DIR}any-arch/sys/embobj/core-vee/src/)
#set(IPNET_FOLDER      ${EO_BASE_DIR}any-arch/sys/embobj/ipnet/src/)
#set(PROT_FOLDER     ${EO_BASE_DIR}any-arch/sys/embobj/prot/src/)
#set(HAL_API_FOLDER    ${EO_BASE_DIR}any-arch/sys/abslayer/api/)
#set(PORTING_FOLDER    ${EO_BASE_DIR}porting/)

if (ICUB_ICUBINTERFACE_EXPERIMENTAL)
    set(ICUB_INTERFACE_CORE_FOLDER  ${CMAKE_SOURCE_DIR}/src/core/iCubInterface-branch/)
else()
    set(ICUB_INTERFACE_CORE_FOLDER  ${CMAKE_SOURCE_DIR}/src/core/iCubInterface/)
endif()

set(PATH_TO_EP_SRC any-arch/sys/embobj/)
set(PATH_TO_CALLBACK ${CMAKE_SOURCE_DIR}/src/libraries/icubmod/)

# schede varie
set(NVS_TYPES       ${EO_BASE_DIR}/${PATH_TO_EP_SRC}/icub/api/)
set(NVS_EB_SRC_FOLDER     ${EO_BASE_DIR}/${PATH_TO_EP_SRC}/icub-nvscfg/board-eps/)


# set(NVS_MC_SRC_FOLDER     ${EO_BASE_DIR}/${PATH_TO_EP_SRC}/icub-nvscfg/ep-motioncontrol/src/)
# set(NVS_MC_API_FOLDER     ${EO_BASE_DIR}/${PATH_TO_EP_SRC}/icub-nvscfg/ep-motioncontrol/api/)
# set(NVS_MC_CBK_FOLDER     ${PATH_TO_CALLBACK}/embObjMotionControl/usrcbk/)
#
# set(NVS_MNGMNT_SRC_FOLDER ${EO_BASE_DIR}/${PATH_TO_EP_SRC}/icub-nvscfg/ep-management/src/)
# set(NVS_MNGMNT_API_FOLDER ${EO_BASE_DIR}/${PATH_TO_EP_SRC}/icub-nvscfg/ep-management/api/)
# set(NVS_MNGMNT_CBK_FOLDER ${PATH_TO_CALLBACK}/embObjLib/usrcbk/)
#
# set(NVS_SENSORS_SRC_FOLDER  ${EO_BASE_DIR}/${PATH_TO_EP_SRC}/icub-nvscfg/ep-analogsensors/src/)
# set(NVS_SENSORS_API_FOLDER  ${EO_BASE_DIR}/${PATH_TO_EP_SRC}/icub-nvscfg/ep-analogsensors/api/)

#
# set(NVS_SKIN_SRC_FOLDER   ${EO_BASE_DIR}/${PATH_TO_EP_SRC}/icub-nvscfg/ep-skin/src/)
# set(NVS_SKIN_API_FOLDER   ${EO_BASE_DIR}/${PATH_TO_EP_SRC}/icub-nvscfg/ep-skin/api/)
# set(NVS_SKIN_CBK_FOLDER   ${PATH_TO_CALLBACK}/embObjSkin/usrcbk/)

# set(NVS_USR_SRC_FOLDER    ${EO_BASE_DIR}/${PATH_TO_EP_SRC}/icub-nvscfg/endpoints/usr-callbacks/src/)
# set(NVS_USR_API_FOLDER    ${EO_BASE_DIR}/${PATH_TO_EP_SRC}/icub-nvscfg/endpoints/usr-callbacks/api/)
# set(NVS_USR_CBK_FOLDER    ${EO_BASE_DIR}/${PATH_TO_CALLBACK}/icub-nvscfg/endpoints/usr-callbacks/usrcbk/)

#--------------------------------------------------------#

set(EXTRA_FOLDER  	    ${CMAKE_CURRENT_SOURCE_DIR}/)
set(TRANSCEIVER_FOLDER	${CMAKE_SOURCE_DIR}/src/libraries/icubmod/embObjLib/ )

#add_definitions( -DMYDEFINE=CIAO )



# using embObjLib.conf...
 include(embObjLib.conf)

  include_directories (  ${PATH_TO_CALLBACK}/embObjAnalog/usrcbk/)
  
# Debug messages
#  message(INFO " lib_cmake - embObj_includes: ${embObj_includes}")
  	
# get all files needed, collected in different lists for better organization and handling

# Common, architecture independent
set(BODY_COMMON_SOURCE 		${BODY_COMMON_FOLDER}/eEcommon.c)
set(BODY_COMMON_HEADER 		${BODY_COMMON_FOLDER}/eEcommon.h)

set(ICUB_INTERFACE_SOURCE	${EXTRA_FOLDER}/FeatureInterface.cpp
#							${ICUB_INTERFACE_CORE_FOLDER}RobotInterfaceRemap.cpp
							)

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


#set(NVS_EB_HEADER 			${NVS_EB_FOLDER}eOcfg_EPs_eb1.h  eOcfg_EPs_eb3.h  eOcfg_EPs_eb5.h  eOcfg_EPs_eb7.h  eOcfg_EPs_eb9.h  _eOcfg_EPs_loc_ebx.h 		eOcfg_EPs_eb2.h  eOcfg_EPs_eb4.h  eOcfg_EPs_eb6.h  eOcfg_EPs_eb8.h  eOcfg_EPs_ebx.h  _eOcfg_EPs_rem_ebx.h)

	
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
#							${NVS_SENSORS_SRC_FOLDER}eOcfg_nvsEP_as_upperarm_con.c 
#							${NVS_SENSORS_SRC_FOLDER}eOcfg_nvsEP_as_upperarm_usr.c 
#							${NVS_SENSORS_SRC_FOLDER}eOcfg_nvsEP_as_wholebody_con.c 
#							${NVS_SENSORS_SRC_FOLDER}eOcfg_nvsEP_as_wholebody_usr.c 
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
							${CORE_YEE}/src/EOYtheSystem.c
							
							)

set(CORE_VEE_SOURCE			${CORE_VEE_FOLDER}/EOVmutex.c
							${CORE_VEE_FOLDER}/EOVtask.c
							${CORE_VEE_FOLDER}/EOVtheSystem.c
#							${CORE_API}/EOVmutex_hid.h
#							${CORE_API}/EOVstorage_hid.h
#							${CORE_API}/EOVtask_hid.h
#							${CORE_API}/EOVtheSystem_hid.h
							)
						
set(CORE_API_HEADER	  			${CORE_API}/EOarray.h
						${CORE_API}/EoCommon.h
						${CORE_API}/EOconstarray.h
						${CORE_API}/EOconstvector.h
						${CORE_API}/EOdeque.h
						${CORE_API}/EOfifo.h
						${CORE_API}/EOfifoByte.h
						${CORE_API}/EOfifoWord.h
						${CORE_API}/EOlist.h
						${CORE_API}/EOtheErrorManager.h
						${CORE_API}/EOtheMemoryPool.h
						${CORE_API}/EOvector.h
#						${CORE_API}/EOVmutex.h
#						${CORE_API}/EOVstorage.h
#						${CORE_API}/EOVtask.h
#						${CORE_API}/EOVtheSystem.h
            ${CORE_YEE}/api/EOYtheSystem.h
						)						

						
set(CORE_FOLDER_HEADER	  			${CORE_API}/EOarray_hid.h
						${CORE_API}/EoCommon_hid.h
						${CORE_API}/EOconstarray_hid.h
						${CORE_API}/EOconstvector_hid.h
						${CORE_API}/EOdeque_hid.h
						${CORE_API}/EOfifo_hid.h
						${CORE_API}/EOfifoByte_hid.h
						${CORE_API}/EOfifoWord_hid.h
						${CORE_API}/EOlist_hid.h
						${CORE_API}/EOtheErrorManager_hid.h
						${CORE_API}/EOtheMemoryPool_hid.h
						${CORE_API}/EOvector_hid.h
						${CORE_API}/EOYtheSystem_hid.h
						)

# EmbObj-NV & Protocol files
set(PROT_SOURCE  				
						${PROT_FOLDER}/EOnv.c
						${PROT_FOLDER}/EOnvsCfg.c
						${PROT_FOLDER}/EOreceiver.c
						${PROT_FOLDER}/EOrop.c
						${PROT_FOLDER}/EOropframe.c
						${PROT_FOLDER}/EOtheAgent.c
						${PROT_FOLDER}/EOtheFormer.c
						${PROT_FOLDER}/EOtheParser.c
						${PROT_FOLDER}/EOtransceiver.c
						${PROT_FOLDER}/EOtransmitter.c
						${PROT_FOLDER}/EOtreenode.c
						${PROT_FOLDER}/EOtheBOARDtransceiver.c
						${PROT_FOLDER}/EOhostTransceiver.c
						${PROT_FOLDER}/EOVstorage.c
						${PROT_FOLDER}EOmatrix3d.c)

set(PROT_HEADER  		${PROT_FOLDER}/EOhostTransceiver_hid.h
						${PROT_FOLDER}/EOnv_hid.h
						${PROT_FOLDER}/EOnvsCfg_hid.h
						${PROT_FOLDER}/EOreceiver_hid.h
						${PROT_FOLDER}/EOrop_hid.h
						${PROT_FOLDER}/EOropframe_hid.h
						${PROT_FOLDER}/EOtheAgent_hid.h
						${PROT_FOLDER}/EOtheFormer_hid.h
						${PROT_FOLDER}/EOtheParser_hid.h
						${PROT_FOLDER}/EOtransceiver_hid.h
						${PROT_FOLDER}/EOtransmitter_hid.h
						${PROT_FOLDER}/EOtreenode_hid.h)


set(IPNET_SOURCE		${IPNET_FOLDER}/EOpacket.c)
set(IPNET_HEADER		${IPNET_FOLDER}/EOpacket_hid.h)


set(HAL_FAKE_SOURCE		${PORTING_FOLDER}/hal_fake.c)

set(EXTRA_SOURCE 		${EXTRA_FOLDER}/hostTransceiver.cpp
						${EXTRA_FOLDER}/ethManager.cpp
#						${EXTRA_FOLDER}/tools/debugFunctions.cpp
						${EXTRA_FOLDER}/eoRequestsQueue.cpp)

set(embobj_source 		${BODY_COMMON_SOURCE} 
						${CORE_FOLDER_SOURCE} 
						${CORE_VEE_SOURCE}  
						${PROT_SOURCE} 
						${IPNET_SOURCE} 
						${HAL_FAKE_SOURCE} 
						${EXTRA_SOURCE}
						${PROT_FOLDER}
						${NVS_EB_SOURCE} 
						${NVS_MC_SOURCE} 
						${NVS_MNGMNT_SOURCE} 
						${NVS_SENSORS_SOURCE} 
						${NVS_SKIN_SOURCE} 
						${NVS_USR_SOURCE} 
						${NVS_CBK_SOURCE}
						${ICUB_INTERFACE_SOURCE})

set(embobj_header 		${BODY_COMMON_SOURCE} 
						${CORE_API_HEADER} 
						${PROT_HEADER} 
						${IPNET_HEADER}
						${NVS_EB_SRC_FOLDER})
					


SOURCE_GROUP("EmbObj Source Files" FILES ${embobj_source})
SOURCE_GROUP("EmbObj Header Files" FILES ${embobj_header})
  
include_directories (   ${CMAKE_CURRENT_SOURCE_DIR}
                        ${embObj_includes}
                        ${iCubDev_INCLUDE_DIRS}
                        ${YARP_INCLUDE_DIRS})
  
add_library(${PROJECTNAME} ${embobj_source} ${embobj_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} debugStream ${YARP_LIBRARIES} ${ACE_LIBRARIES} )  

icub_export_library(${PROJECTNAME})
