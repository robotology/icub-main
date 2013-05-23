# Copyright: (C) 2010 RobotCub Consortium
# Authors: Valentina Gaggero
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)

set(PROJECTNAME emsBackDoorManager)

file(GLOB folder_header *.h)
file(GLOB folder_source *.cpp)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

set(EXTRA_FOLDER  		${CMAKE_CURRENT_SOURCE_DIR}/)



# lib
SET(ACE_LIBRARIES 	/usr/lib/libACE.so)
message(INFO " lib ace -${ACE_LIBRARIES}")




#buttons file cfg
IF ("${BUTTON_FILE}" STREQUAL "")
  MESSAGE(STATUS "No button file")
ELSE ("${BUTTON_FILE}" STREQUAL "")
    SET(BUTTON_FILE "${BUTTON_FILE}")
  IF (EXISTS ${BUTTON_FILE})
        SET(MY_CALLBACK ${BUTTON_FILE})
        MESSAGE(STATUS " Button file ${BUTTON_FILE}")
    ELSE ("${BUTTON_FILE}" STREQUAL "")
        MESSAGE(ERROR " Button file ${BUTTON_FILE} not found")
    ENDIF(EXISTS ${BUTTON_FILE})
ENDIF ("${BUTTON_FILE}" STREQUAL "")



#headers-----------

SET(ACE_INCLUDE_DIR "/usr/include")

#set(BASE 			$ENV{ICUB_ROOT}/firmware/emBODY/)
set(BASE 			$ENV{ICUB_ROOT}/main/src/libraries/icubmod/emBODY/)
set(OPC_PROTO_FOLDER             ${BASE}embobj/icub-app/)
set(COMMON_FOLDER		 ${BASE}embobj/core/)
set(PORTING_FOLDER		 ${BASE}/porting/)
set(EMBOBJ_TYPE_FOLDER           ${BASE}embobj/icub/api/)

set(ALL_INCLUDES				${EXTRA_FOLDER}
						${ACE_INCLUDE_DIR}
						${OPC_PROTO_FOLDER}/api
						${OPC_PROTO_FOLDER}/src
						${OPC_PROTO_FOLDER}/cfg
						${COMMON_FOLDER}/api
						${PORTING_FOLDER}
						${EMBOBJ_TYPE_FOLDER}
)

include_directories (${ALL_INCLUDES} )
message(INFO "includes: ${ALL_INCLUDES}")

#source code --------------
#set(BODY_COMMON_SOURCE 		${BODY_COMMON_FOLDER}/eEcommon.c)


set(OPC_PROTO_HEADER		${OPC_PROTO_FOLDER}/api/OPCprotocolManager.h
				)
message(INFO "OPC_PROTO_HEADER: ${OPC_PROTO_HEADER}")


set(OPC_PROTO_SOURCE
					${OPC_PROTO_FOLDER}/src/OPCprotocolManager.c
					${OPC_PROTO_FOLDER}/src/OPCprotocolManager_hid.h
					)

# configuration ------------------------
set(OPC_PROTO_CFG_HEADER		${OPC_PROTO_FOLDER}/cfg/OPCprotocolManager_Cfg.h
					)
set(OPC_PROTO_CFG_SOURCE		${OPC_PROTO_FOLDER}/cfg/OPCprotocolManager_Cfg.c
					)


set(embobj_source 
					${OPC_PROTO_SOURCE}
					${OPC_PROTO_CFG_SOURCE}
					)



set(EXTRA_SOURCE 				${EXTRA_FOLDER}/main.cpp
						${EXTRA_FOLDER}/diagnostics_buttons.cpp
						${EXTRA_FOLDER}/opcManager_cfg4pc104.c
						)

set(ALL_SOURCE_CODE				${embobj_source}
						${EXTRA_SOURCE})

add_executable(${PROJECTNAME} ${ALL_SOURCE_CODE})
target_link_libraries(${PROJECTNAME} ${ACE_LIBRARIES} pthread)
#target_link_libraries(${PROJECTNAME} pcap)

