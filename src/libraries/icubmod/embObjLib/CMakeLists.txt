# Create a library for all embObj stuff

SET(PROJECTNAME hostTransceiver)
PROJECT(${PROJECTNAME})

PREPARE_DEVICE(${PROJECTNAME} TYPE ${PROJECTNAME}
    INCLUDE hostTransceiver.hpp
    WRAPPER " ")

set(BASE $ENV{ICUB_ROOT}/firmware/emBODY/)
set(EXTRA_FOLDER  	${CMAKE_CURRENT_SOURCE_DIR}/)

include(embObjLib.conf)

# Debug messages
# message(INFO " BASE=${BASE}")
# message(INFO " lib_cmake - embObj_includes: ${embObj_includes}")
  	
# get all files needed, collected in different lists for better organization and handling

# Common, architecture independent
set(BODY_COMMON_SOURCE 		${BODY_COMMON_FOLDER}/eEcommon.c)
set(BODY_COMMON_HEADER 		${BODY_COMMON_FOLDER}/eEcommon.h)

# Network Variable configuration
set(NVS_COMMON_SOURCE 		${NVS_COMMON_FOLDER}/eOcfg_nvsEP_base_con.c
							${NVS_COMMON_FOLDER}/eOcfg_nvsEP_mngmnt_con.c
							${NVS_COMMON_FOLDER}/eOcfg_nvsEP_joint_con.c)

set(NVS_COMMON_HEADER 		${NVS_COMMON_FOLDER}/eOcfg_nvsEP_base_con.h
							${NVS_COMMON_FOLDER}/eOcfg_nvsEP_mngmnt_con.h
							${NVS_COMMON_FOLDER}/eOcfg_nvsEP_joint_con.h)

set(NVS_REMOTE_SOURCE		${NVS_REMOTE_FOLDER}/eOcfg_EPs_rem_board.c
							${NVS_REMOTE_FOLDER}/eOcfg_nvsEP_base_usr_rem_anydev.c
							${NVS_REMOTE_FOLDER}/eOcfg_nvsEP_mngmnt_usr_rem_board.c
							${NVS_REMOTE_FOLDER}/eOcfg_nvsEP_joint_usr_rem_board.c)

set(NVS_REMOTE_HEADER		${NVS_REMOTE_FOLDER}/eOcfg_EPs_rem_board.h
							${NVS_REMOTE_FOLDER}/eOcfg_nvsEP_base_usr_rem_anydev.h
							${NVS_REMOTE_FOLDER}/eOcfg_nvsEP_mngmnt_usr_rem_board.h
							${NVS_REMOTE_FOLDER}/eOcfg_nvsEP_joint_usr_rem_board.h)

set(NVS_MC_EP_SOURCE 		${NVS_MC_EP}eOcfg_nvsEP_mc_any_con_bodypart.c
							${NVS_MC_EP}eOcfg_nvsEP_mc_any_con_jxxdefault.c
							${NVS_MC_EP}eOcfg_nvsEP_mc_any_con_mxxdefault.c
							${NVS_MC_EP}eOcfg_nvsEP_mc_leftleg_con.c
							${NVS_MC_EP}eOcfg_nvsEP_mc_leftleg_usr_rem_ebx.c
																			)

set(NVS_MC_EP_HEADER 		${NVS_MC_EP}/eOcfg_nvsEP_mc_any_con_bodypart.h
							${NVS_MC_EP}/eOcfg_nvsEP_mc_any_con_jxx.h
							${NVS_MC_EP}/eOcfg_nvsEP_mc_any_con_jxxdefault.h
							${NVS_MC_EP}/eOcfg_nvsEP_mc_any_con_mxx.h
							${NVS_MC_EP}/eOcfg_nvsEP_mc_any_con_mxxdefault.h
							${NVS_MC_EP}/eOcfg_nvsEP_mc_leftleg_con.h
							${NVS_MC_EP}/eOcfg_nvsEP_mc_leftleg_usr_rem_ebx.h)


# EmbObj-src
set(CORE_FOLDER_SOURCE  			${CORE_FOLDER}/EOarray.c
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
						${CORE_FOLDER}/EOvector.c)

set(CORE_VEE_SOURCE				${CORE_VEE_FOLDER}/EOVmutex.c
						${CORE_VEE_FOLDER}/EOVtask.c
						${CORE_VEE_FOLDER}/EOVtheSystem.c
#						${CORE_API}/EOVmutex_hid.h
#						${CORE_API}/EOVstorage_hid.h
#						${CORE_API}/EOVtask_hid.h
#						${CORE_API}/EOVtheSystem_hid.h
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

set(PROT_HEADER  				${PROT_FOLDER}/EOhostTransceiver_hid.h
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


set(IPNET_SOURCE				${IPNET_FOLDER}/EOpacket.c)

set(IPNET_HEADER				${IPNET_FOLDER}/EOpacket_hid.h)


set(HAL_FAKE_SOURCE		${PORTING_FOLDER}/hal_fake.c)

set(EXTRA_SOURCE 		${EXTRA_FOLDER}/hostTransceiver.cpp)

set(embobj_source 		${BODY_COMMON_SOURCE} 
				${CORE_FOLDER_SOURCE} 
				${CORE_VEE_SOURCE}  
				${PROT_SOURCE} 
				${IPNET_SOURCE} 
				${HAL_FAKE_SOURCE} 
				${EXTRA_SOURCE}
				${PROT_FOLDER}
				${NVS_COMMON_SOURCE} 
				${NVS_REMOTE_SOURCE}
				${NVS_MC_EP_SOURCE})


set(embobj_header 		${BODY_COMMON_SOURCE} 
						${CORE_API_HEADER} 
						${PROT_HEADER} 
						${IPNET_HEADER}
						${NVS_COMMON_HEADER} 
						${NVS_REMOTE_HEADER}
						${NVS_MC_EP_HEADER})

SOURCE_GROUP("EmbObj Source Files" FILES ${folder_source})
SOURCE_GROUP("EmbObj Header Files" FILES ${folder_header})
  
add_library(${PROJECTNAME} ${embobj_source} ${embobj_header})

icub_export_library(${PROJECTNAME})
