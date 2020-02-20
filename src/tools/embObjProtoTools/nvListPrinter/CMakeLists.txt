# Copyright: (C) 2010 RobotCub Consortium
# Authors: Valentina Gaggero
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)

set(PROJECTNAME nvid_printer)

file(GLOB folder_header *.h)
file(GLOB folder_source *.cpp)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

set(EXTRA_FOLDER  		${CMAKE_CURRENT_SOURCE_DIR}/)


#headers-----------
message(INFO " ciao -${folder_source}")

#set(BASE 			$ENV{ICUB_ROOT}/firmware/emBODY/)
set(BASE 			$ENV{ICUB_ROOT}/main/src/libraries/icubmod/emBODY/)
set(MIDDLE_PATH_TO_NVS 		any-arch/sys/embobj/)


set(BODY_COMMON_FOLDER  	${BASE}any-arch/body/embenv/envcom/)
set(CORE_FOLDER  		${BASE}any-arch/sys/embobj/core/src/)
set(CORE_API	  		${BASE}any-arch/sys/embobj/core/api/)
set(CORE_VEE_FOLDER  	        ${BASE}any-arch/sys/embobj/core-vee/src/)
set(PROT_FOLDER  		${BASE}any-arch/sys/embobj/prot/src/)
set(PORTING_FOLDER 		${BASE}porting/)
set(BODY_COMMON_HEADER 		${BODY_COMMON_FOLDER}/eEcommon.h)
set(IPNET_FOLDER  		${BASE}any-arch/sys/embobj/ipnet/src/)
set(UTILS_FOLDER  		${BASE}any-arch/sys/embobj/utils/src/)


# schede varie
set(NVS_TYPES			${BASE}/${MIDDLE_PATH_TO_NVS}/icub/api/)
#set(NVS_EB_SRC_FOLDER  		${BASE}/${MIDDLE_PATH_TO_NVS}/icub-nvscfg/board-eps/)
#set(NVS_EB_API_FOLDER  	       ${BASE}/${MIDDLE_PATH_TO_NVS}/icub-nvscfg/cfg-eps-ebx/api/)
#set(NVS_EB_CBK_FOLDER  	${BASE}/${DEMO_PATH_TO_NVS}/icub-nvscfg/cfg-eps-ebx/usrcbk/)

set(NVS_MC_SRC_FOLDER  		${BASE}/${MIDDLE_PATH_TO_NVS}/icub-nvscfg/ep-motioncontrol/src/)
set(NVS_MC_API_FOLDER  		${BASE}/${MIDDLE_PATH_TO_NVS}/icub-nvscfg/ep-motioncontrol/api/)
#set(NVS_MC_CBK_FOLDER  		${BASE}/${DEMO_PATH_TO_NVS}/icub-nvscfg/ep-motioncontrol/usrcbk/)

set(NVS_MNGMNT_SRC_FOLDER	${BASE}/${MIDDLE_PATH_TO_NVS}/icub-nvscfg/ep-management/src/)
set(NVS_MNGMNT_API_FOLDER	${BASE}/${MIDDLE_PATH_TO_NVS}/icub-nvscfg/ep-management/api/)
#set(NVS_MNGMNT_CBK_FOLDER	${BASE}/${DEMO_PATH_TO_NVS}/icub-nvscfg/ep-management/usrcbk/)

set(NVS_SENSORS_SRC_FOLDER  ${BASE}/${MIDDLE_PATH_TO_NVS}/icub-nvscfg/ep-analogsensors/src/)
set(NVS_SENSORS_API_FOLDER  ${BASE}/${MIDDLE_PATH_TO_NVS}/icub-nvscfg/ep-analogsensors/api/)
#set(NVS_SENSORS_CBK_FOLDER  ${BASE}/${DEMO_PATH_TO_NVS}/icub-nvscfg/ep-analogsensors/usrcbk/)

set(NVS_SKIN_SRC_FOLDER  	${BASE}/${MIDDLE_PATH_TO_NVS}/icub-nvscfg/ep-skin/src/)
set(NVS_SKIN_API_FOLDER  	${BASE}/${MIDDLE_PATH_TO_NVS}/icub-nvscfg/ep-skin/api/)
#set(NVS_SKIN_CBK_FOLDER  	${BASE}/${DEMO_PATH_TO_NVS}/icub-nvscfg/ep-skin/usrcbk/)



set(embObj_includes 				${BODY_COMMON_FOLDER}
						${CORE_API}
						${CORE_FOLDER}
						${PROT_FOLDER}../api/
						${PROT_FOLDER}
						${BASE}../porting/
						${PORTING_FOLDER}
						${CORE_VEE_FOLDER}../api/
						${CORE_VEE_FOLDER}
						${IPNET_FOLDER}../api/
						${IPNET_FOLDER}
						${UTILS_FOLDER}
						${UTILS_FOLDER}../api/
						${NVS_TYPES}
						${NVS_EB_API_FOLDER}
						${NVS_MC_API_FOLDER}
						${NVS_MNGMNT_API_FOLDER}
						${NVS_SENSORS_API_FOLDER}
						${NVS_SKIN_API_FOLDER}
						${NVS_EB_SRC_FOLDER}
						${NVS_MC_SRC_FOLDER}
						${NVS_MNGMNT_SRC_FOLDER}
						${NVS_SENSORS_SRC_FOLDER}
						${NVS_SKIN_SRC_FOLDER})

set(ALL_INCLUDES				${embObj_includes}
						${EXTRA_FOLDER})

include_directories (${ALL_INCLUDES} )
message(INFO "includes: ${ALL_INCLUDES}")

#source code --------------
set(BODY_COMMON_SOURCE 		${BODY_COMMON_FOLDER}/eEcommon.c)

set(CORE_FOLDER_SOURCE  				${CORE_FOLDER}/EOarray.c
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


set(CORE_VEE_SOURCE					${CORE_VEE_FOLDER}/EOVmutex.c
							${CORE_VEE_FOLDER}/EOVtask.c
							${CORE_VEE_FOLDER}/EOVtheSystem.c	)


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



set(IPNET_SOURCE		${IPNET_FOLDER}/EOpacket.c)
set(IPNET_HEADER		${IPNET_FOLDER}/EOpacket_hid.h)


set(UTILS_SOURCE		${UTILS_FOLDER}/eODeb_eoProtoParser.c
				${UTILS_FOLDER}/eOtheEthLowLevelParser.c)

set(UTILS_HEADER		${UTILS_FOLDER}/eODeb_eoProtoParser_hid.h
				${UTILS_FOLDER}/eOtheEthLowLevelParser_hid.h)


# Network Variable configuration
set(NVS_EB_SOURCE 					${NVS_EB_SRC_FOLDER}eOcfg_EPs_board.c
                            				${NVS_EB_SRC_FOLDER}eOcfg_EPs_eb1.c
							${NVS_EB_SRC_FOLDER}eOcfg_EPs_eb2.c
							${NVS_EB_SRC_FOLDER}eOcfg_EPs_eb3.c
							${NVS_EB_SRC_FOLDER}eOcfg_EPs_eb4.c
							${NVS_EB_SRC_FOLDER}eOcfg_EPs_eb5.c
							${NVS_EB_SRC_FOLDER}eOcfg_EPs_eb6.c
							${NVS_EB_SRC_FOLDER}eOcfg_EPs_eb7.c
							${NVS_EB_SRC_FOLDER}eOcfg_EPs_eb8.c
							${NVS_EB_SRC_FOLDER}eOcfg_EPs_eb9.c  )





set(NVS_MC_SOURCE 					${NVS_MC_SRC_FOLDER}eOcfg_nvsEP_mc.c
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

set(NVS_MNGMNT_SOURCE					${NVS_MNGMNT_SRC_FOLDER}eOcfg_nvsEP_mn.c
							${NVS_MNGMNT_SRC_FOLDER}eOcfg_nvsEP_mn_appl_con.c
							${NVS_MNGMNT_SRC_FOLDER}eOcfg_nvsEP_mn_appl_usr.c
							${NVS_MNGMNT_SRC_FOLDER}eOcfg_nvsEP_mn_comm_con.c
							${NVS_MNGMNT_SRC_FOLDER}eOcfg_nvsEP_mn_comm_usr.c )

set(NVS_SENSORS_SOURCE					${NVS_SENSORS_SRC_FOLDER}eOcfg_nvsEP_as.c
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



set(NVS_SKIN_SOURCE			${NVS_SKIN_SRC_FOLDER}eOcfg_nvsEP_sk.c
							${NVS_SKIN_SRC_FOLDER}eOcfg_nvsEP_sk_emsboard_con.c
							${NVS_SKIN_SRC_FOLDER}eOcfg_nvsEP_sk_emsboard_usr.c )


set(NVS_CBK_SOURCE			${NVS_MNGMNT_CBK_FOLDER}eOcfg_nvsEP_mngmnt_usr_ebx.c
							${NVS_SKIN_CBK_FOLDER}eOcfg_nvsEP_sk_usrcbk_ebx.c
							${NVS_MC_CBK_FOLDER}eOcfg_nvsEP_mc_usrcbk_pippo.c
							${NVS_SENSORS_CBK_FOLDER}eOcfg_nvsEP_as_usrcbk_pippo.c
							)





set(embobj_source 				${BODY_COMMON_SOURCE}
						${CORE_FOLDER_SOURCE}
						${PROT_SOURCE}
						${PROT_HEADER}
						${PROT_FOLDER}
						${CORE_VEE_SOURCE}
						${IPNET_SOURCE}
						${UTILS_HEADER}
						${UTILS_SOURCE}
#						${NVS_EB_SOURCE}
#						${NVS_MC_SOURCE}
#						${NVS_MNGMNT_SOURCE}
#						${NVS_SENSORS_SOURCE}
#						${NVS_CBK_SOURCE}
#						${NVS_USR_SOURCE}
#						${NVS_SKIN_SOURCE}
						)



set(EXTRA_SOURCE 				${EXTRA_FOLDER}/nvid_printer.cpp)

set(ALL_SOURCE_CODE				${embobj_source}
						${EXTRA_SOURCE})

add_executable(${PROJECTNAME} ${ALL_SOURCE_CODE})

#target_link_libraries(${PROJECTNAME} pcap)

