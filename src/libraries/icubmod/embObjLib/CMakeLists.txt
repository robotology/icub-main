# Copyright: (C) 2012 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Name of the project must correspond to the class name that needs to be
# instantiated by the factory - so embObjLib is not ok
project(ethResources)



if(ICUB_COMPILE_EMBOBJ_LIBRARY)

if(NOT ICUB_HAS_icub_firmware_shared)
  message(ERROR " embObj: cannot find icub_firmware_shared library, turn off device embObjDevices
  embObj library can now be found in the icub-firmware-shared package that
  can be downloaded from here:
  .
       https://github.com/robotology/icub-firmware-shared
  .    ")
else()
# Debug messages
#  message(INFO " lib_cmake - embObj_includes: ${embObj_includes}")

# get all files needed, collected in different lists for better organization and handling

# Common, architecture independent


set(ICUB_INTERFACE_SOURCE   ${CMAKE_CURRENT_SOURCE_DIR}/FeatureInterface.cpp)
set(EXTRA_SOURCE            ${CMAKE_CURRENT_SOURCE_DIR}/hostTransceiver.cpp
                            ${CMAKE_CURRENT_SOURCE_DIR}/ethManager.cpp
                            ${CMAKE_CURRENT_SOURCE_DIR}/ethResource.cpp
                            ${CMAKE_CURRENT_SOURCE_DIR}/ethMonitorPresence.cpp
                            ${CMAKE_CURRENT_SOURCE_DIR}/ethBoards.cpp
                            ${CMAKE_CURRENT_SOURCE_DIR}/ethSender.cpp
                            ${CMAKE_CURRENT_SOURCE_DIR}/ethReceiver.cpp
                            ${CMAKE_CURRENT_SOURCE_DIR}/ethParser.cpp
                            ${CMAKE_CURRENT_SOURCE_DIR}/IethResource.cpp
                            ${CMAKE_CURRENT_SOURCE_DIR}/fakeEthResource.cpp
                            ${CMAKE_CURRENT_SOURCE_DIR}/serviceParser.cpp
                            ${CMAKE_CURRENT_SOURCE_DIR}/theNVmanager.cpp
                            ${CMAKE_CURRENT_SOURCE_DIR}/embObjGeneralDevPrivData.cpp
                            )
set(NVS_CBK_SOURCE  ${CMAKE_CURRENT_SOURCE_DIR}/protocolCallbacks/EoProtocolMN_fun_userdef.c
                    ${CMAKE_CURRENT_SOURCE_DIR}/protocolCallbacks/EoProtocolMC_fun_userdef.c
                    ${CMAKE_CURRENT_SOURCE_DIR}/protocolCallbacks/EoProtocolAS_fun_userdef.c
                    ${CMAKE_CURRENT_SOURCE_DIR}/protocolCallbacks/EoProtocolSK_fun_userdef.c)


set(DEBUG_SOURCE            ${DEBUG_FOLDER}/eODeb_eoProtoParser.c
                            ${DEBUG_FOLDER}/eOtheEthLowLevelParser.c)

option(NETWORK_PERFORMANCE_BENCHMARK "Enable embedded network perfomance bamchmark." OFF)
mark_as_advanced(NETWORK_PERFORMANCE_BENCHMARK)

if(NETWORK_PERFORMANCE_BENCHMARK)

set(TOOLS_FOLDER    ${CMAKE_CURRENT_SOURCE_DIR}/tools)
set(TOOLS_HEADER    ${TOOLS_FOLDER}/include/PeriodicEventsVerifier.h)
set(TOOLS_SOURCE    ${TOOLS_FOLDER}/src/PeriodicEventsVerifier.cpp 
                    ${TOOLS_FOLDER}/src/embot_tools.cpp)
else()

set(TOOLS_FOLDER  ""  )
set(TOOLS_HEADER  ""  )
set(TOOLS_SOURCE  ""  )

endif()



set(embobj_source           ${EXTRA_SOURCE}
                            ${ICUB_INTERFACE_SOURCE}
                            ${TOOLS_SOURCE}
                            ${NVS_CBK_SOURCE})

set(embobj_header           ${TOOLS_HEADER} )

SOURCE_GROUP("EmbObj Source Files" FILES ${embobj_source})
SOURCE_GROUP("EmbObj Header Files" FILES ${embobj_header})

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
add_library(${PROJECT_NAME} SHARED ${embobj_source} ${embobj_header})

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../motionControlLib
                                                ${PATH_TO_CALLBACK}/embObjAnalog/usrcbk/
                                                ../skinLib)
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                                                  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

target_link_libraries(${PROJECT_NAME} YARP::YARP_os
                                      icub_firmware_shared::embobj
                                      ACE::ACE)

icub_export_library(${PROJECT_NAME})

endif(NOT ICUB_HAS_icub_firmware_shared)

endif(ICUB_COMPILE_EMBOBJ_LIBRARY)

