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




FIND_PACKAGE(icub_firmware_shared)


if(icub_firmware_shared_embobj_FOUND)
    set(BASE         ${icub_firmware_shared_embobj_INCLUDE_DIR})
else()
    set(BASE         $ENV{ICUB_ROOT}/src/libraries/icubmod/embObjLib/embObj/embobj/)
endif()

set(EOPROT_USE_MN_VERSION_2_0 ON)

add_definitions(-DEOPROT_USE_MN_VERSION_2_0)

set(CORE_FOLDER         ${BASE}/embobj/core/core/)
set(CORE_API            ${BASE}/embobj/core/core/)
set(PROT_FOLDER         ${BASE}/embobj/plus/comm-v2/)
set(UTILS_FOLDER        ${BASE}/embobj/plus/utils/)
set(ICUB_API            ${BASE}/embobj/plus/comm-v2/icub/)


set(embObj_includes     
                        ${CORE_FOLDER}
			${ICUB_API}
                        ${PROT_FOLDER}/transport/
                        ${PROT_FOLDER}/protocol/api
                        ${UTILS_FOLDER}
                        )

set(ALL_INCLUDES        ${embObj_includes}
                        ${EXTRA_FOLDER}
                        )

include_directories (${ALL_INCLUDES} )
#message(INFO "includes: ${ALL_INCLUDES}")

#source code --------------

set(UTILS_SOURCE        ${UTILS_FOLDER}/eODeb_eoProtoParser.c
                        ${UTILS_FOLDER}/eOtheEthLowLevelParser.c
                        )

set(UTILS_HEADER        ${UTILS_FOLDER}/eODeb_eoProtoParser_hid.h
                        ${UTILS_FOLDER}/eOtheEthLowLevelParser_hid.h
                        )



set(CORE_SOURCE         ${CORE_FOLDER}/EOarray.c
                        ${CORE_FOLDER}/EOarray_hid.h
                        ${CORE_FOLDER}/EOtheErrorManager.c
                        ${CORE_FOLDER}/EOtheMemoryPool.c
                        ${CORE_FOLDER}/EoCommon.c
                        )

set(CORE_VEE_SOURCE     ${CORE_FOLDER}/EOVmutex.c
                        ${CORE_FOLDER}/EOVtask.c
                        ${CORE_FOLDER}/EOVtheSystem.c	
                        )

#set(PROTO_SRC               ${PROT_FOLDER}/protocol/src/EoProtocol.c)

set(embobj_source 
                        ${CORE_SOURCE}
                        ${UTILS_HEADER}
                        ${UTILS_SOURCE}
                        ${CORE_VEE_SOURCE}
#                        ${PROTO_SRC}
                        )


#insert here name of parser cfg file!!! 
#set(PARSER_CFG_FILE    ${EXTRA_FOLDER}/parser_cfg_getPid.cpp )
#set(PARSER_CFG_FILE    ${EXTRA_FOLDER}/parser_cfg_searchPosSetpoint4all.cpp )
#set(PARSER_CFG_FILE    ${EXTRA_FOLDER}/parser_cfg_searchPosSetpoint.cpp )
#set(PARSER_CFG_FILE    ${EXTRA_FOLDER}/parser_cfg_enaDisaJoints.cpp)
#set(PARSER_CFG_FILE    ${EXTRA_FOLDER}/parser_cfg_startupConfig.cpp)
set(PARSER_CFG_FILE    ${EXTRA_FOLDER}/parser_cfg_fullscale.cpp)
#set(PARSER_CFG_FILE    ${EXTRA_FOLDER}/parser_cfg_seqNum.cpp)
#set(PARSER_CFG_FILE    ${EXTRA_FOLDER}/parser_cfg_posleftlegj0.cpp)

message(INFO "  you are using  ${PARSER_CFG_FILE} for parser cfg!!!")

set(EXTRA_SOURCE 	    ${PARSER_CFG_FILE}                
                        ${EXTRA_FOLDER}/main.cpp
                        ${EXTRA_FOLDER}/pcap_wrapper_linux.cpp
                        )

set(ALL_SOURCE_CODE	    ${embobj_source}
                        ${EXTRA_SOURCE}
                        )

add_executable(${PROJECTNAME} ${ALL_SOURCE_CODE})

target_link_libraries(${PROJECTNAME} pcap)

