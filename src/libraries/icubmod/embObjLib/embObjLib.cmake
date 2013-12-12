# Set some useful variable for compiling the library
# This should be included by any CMakeLists.txt aiming to compile
# source file that needs to use the protocol.

set(EO_BASE_DIR         ${CMAKE_SOURCE_DIR}/src/libraries/icubmod/embObjLib/embObj/)

set(CORE_FOLDER  		${EO_BASE_DIR}/embobj/core/core)
set(CORE_YEE            ${EO_BASE_DIR}/embobj/core/exec/yarp/)
set(PROT_FOLDER  		${EO_BASE_DIR}/embobj/plus/comm-v1/)

SET(DEBUG_FOLDER        ${EO_BASE_DIR}/embobj/plus/utils)

if (ICUB_ICUBINTERFACE_EXPERIMENTAL)
    set(ICUB_INTERFACE_CORE_FOLDER	${CMAKE_SOURCE_DIR}/src/core/iCubInterface-branch/)
else()
    set(ICUB_INTERFACE_CORE_FOLDER  ${CMAKE_SOURCE_DIR}/src/core/iCubInterface/)
endif()

set(PATH_TO_EP_SRC nvs)
set(PATH_TO_CALLBACK ${CMAKE_SOURCE_DIR}/src/libraries/icubmod/)

# schede varie
set(NVS_TYPES				${PROT_FOLDER}/icub/)
set(NVS_EB_SRC_FOLDER  		${PROT_FOLDER}/${PATH_TO_EP_SRC}/board-eps/)


set(NVS_MC_SRC_FOLDER  		${PROT_FOLDER}/${PATH_TO_EP_SRC}/ep-motioncontrol/)
set(NVS_MC_CBK_FOLDER  		${PATH_TO_CALLBACK}/embObjMotionControl/usrcbk/)

set(NVS_MNGMNT_SRC_FOLDER	${PROT_FOLDER}/${PATH_TO_EP_SRC}/ep-management/)
set(NVS_MNGMNT_CBK_FOLDER	${PATH_TO_CALLBACK}/embObjLib/usrcbk/)

set(NVS_SENSORS_SRC_FOLDER  ${PROT_FOLDER}/${PATH_TO_EP_SRC}/ep-analogsensors/)
set(NVS_SENSORS_CBK_FOLDER  ${PATH_TO_CALLBACK}/embObjAnalog/usrcbk/)


set(NVS_SKIN_SRC_FOLDER  	${PROT_FOLDER}/${PATH_TO_EP_SRC}/ep-skin/)
set(NVS_SKIN_CBK_FOLDER  	${PATH_TO_CALLBACK}/embObjSkin/usrcbk/)


set(EXTRA_FOLDER  	    ${CMAKE_CURRENT_SOURCE_DIR}/)
set(TRANSCEIVER_FOLDER	${CMAKE_SOURCE_DIR}/src/libraries/icubmod/embObjLib/ )

set(embObj_includes     ${BODY_COMMON_FOLDER}
                        ${NVS_TYPES}
                        ${NVS_EB_SRC_FOLDER}
                        ${NVS_MC_SRC_FOLDER}
                        ${NVS_MNGMNT_SRC_FOLDER}
                        ${NVS_SENSORS_SRC_FOLDER}
                        ${NVS_SKIN_SRC_FOLDER}
                        ${CORE_YEE}
                        ${CORE_FOLDER}
                        ${PROT_FOLDER}/prot/
                        ${HAL_API_FOLDER}
                        ${EXTRA_FOLDER}
                        ${YARP_INCLUDE_DIRS}
                        ${iCubDev_INCLUDE_DIRS}		#??
                        ${ICUB_INTERFACE_CORE_FOLDER}
                        ${ICUB_INTERFACE_CORE_FOLDER}/robotInterface
                        ${NVS_SKIN_SRC_FOLDER}
                        ${NVS_MNGMNT_CBK_FOLDER}
                        ${NVS_MC_CBK_FOLDER}
                        ${NVS_SKIN_CBK_FOLDER}
                        ${NVS_SENSORS_CBK_FOLDER}
                        ${CMAKE_SOURCE_DIR}/src/libraries/icubmod/embObjLib/
                        ${CMAKE_SOURCE_DIR}/src/libraries/icubmod/embObjLib/tools/
                        ${CMAKE_SOURCE_DIR}/src/libraries/icubmod/embObjMotionControl/
                        ${CMAKE_SOURCE_DIR}/src/libraries/icubmod/embObjSkin/
                        ${CMAKE_SOURCE_DIR}/src/libraries/icubmod/embObjAnalog/
                        ${TRANSCEIVER_FOLDER}/tools/
                        ${ACE_INCLUDE_DIR}
                        ${CMAKE_SOURCE_DIR}/src/libraries/icubmod/debugStream/
                        ${DEBUG_FOLDER}
                        )

include_directories (${embObj_includes})
