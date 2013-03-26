# Copyright: (C) 2010 RobotCub Consortium
# Authors: Valentina Gaggero
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)

set(PROJECTNAME embObjProto_debugParser)

file(GLOB folder_header *.h)
file(GLOB folder_source *.cpp)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

set(EXTRA_FOLDER  		${CMAKE_CURRENT_SOURCE_DIR}/)


#headers-----------
message(INFO " ciao -${folder_source}")

set(BASE 			$ENV{ICUB_ROOT}/firmware/emBODY/)


set(BODY_COMMON_FOLDER  	${BASE}any-arch/body/embenv/envcom/)
set(CORE_FOLDER  		${BASE}any-arch/sys/embobj/core/src/)
set(CORE_API	  		${BASE}any-arch/sys/embobj/core/api/)
set(CORE_VEE_FOLDER  	        ${BASE}any-arch/sys/embobj/core-vee/src/)
set(PROT_FOLDER  		${BASE}any-arch/sys/embobj/prot/src/)
set(PORTING_FOLDER 		${BASE}porting/)
set(BODY_COMMON_HEADER 		${BODY_COMMON_FOLDER}/eEcommon.h)
set(IPNET_FOLDER  		${BASE}any-arch/sys/embobj/ipnet/src/)
set(UTILS_FOLDER  		${BASE}any-arch/sys/embobj/utils/src/)

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
						${UTILS_FOLDER}../api/)   

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

set(embobj_source 				${BODY_COMMON_SOURCE} 
						${CORE_FOLDER_SOURCE} 
						${PROT_SOURCE} 
						${PROT_HEADER} 
						${PROT_FOLDER}
						${CORE_VEE_SOURCE}
						${IPNET_SOURCE}
						${UTILS_HEADER}
						${UTILS_SOURCE})



set(EXTRA_SOURCE 				${EXTRA_FOLDER}/main.cpp
						${EXTRA_FOLDER}/pcap_wrapper_linux.cpp)

set(ALL_SOURCE_CODE				${embobj_source}
						${EXTRA_SOURCE})

add_executable(${PROJECTNAME} ${ALL_SOURCE_CODE})

target_link_libraries(${PROJECTNAME} pcap)

