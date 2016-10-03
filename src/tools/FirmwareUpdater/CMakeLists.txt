# Copyright (C) 2014  iCub Facility, Istituto Italiano di Tecnologia
# Author: Daniele E. Domenichelli <daniele.domenichelli@iit.it>, Marco Acccame <marco.accame@iit.it>
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

include(iCubUseQt5)

set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

include_directories(${YARP_INCLUDE_DIRS})

include_directories(${ACE_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../ethLoader/ethLoaderLib)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../canLoader/canLoaderLib)

include_directories(${icub_firmware_shared_canProtocolLib_INCLUDE_DIR})
include_directories(${icub_firmware_shared_canProtocolLib_INCLUDE_DIR}/canProtocolLib)
include_directories(${icub_firmware_shared_embobj_INCLUDE_DIR}/embobj/plus/comm-v2/icub/)
include_directories(${icub_firmware_shared_embobj_INCLUDE_DIR}/embobj/core/core)


set(FirmwareUpdater_qt_SRCS changeaddressdialog.cpp
                            changeinfodialog.cpp
                            firmwareupdatercore.cpp
                            main.cpp
                            mainwindow.cpp
                            selectioncheckbox.cpp)
set(FirmwareUpdater_qt_HDRS changeaddressdialog.h
                            changeinfodialog.h
                            firmwareupdatercore.h
                            mainwindow.h
                            selectioncheckbox.h)
set(FirmwareUpdater_qt_QRC_FILES res.qrc)
set(FirmwareUpdater_qt_UI_FILES changeaddressdialog.ui
                            changeinfodialog.ui
                            mainwindow.ui)

qt5_add_resources(FirmwareUpdater_qt_QRC_GEN_SRCS ${FirmwareUpdater_qt_QRC_FILES})
qt5_wrap_ui(FirmwareUpdater_qt_UI_GEN_SRCS ${FirmwareUpdater_qt_UI_FILES})

source_group("Source Files" FILES ${FirmwareUpdater_qt_SRCS})
source_group("Header Files" FILES ${FirmwareUpdater_qt_HDRS})
source_group("Resources Files" FILES ${FirmwareUpdater_qt_QRC_FILES})
source_group("UI Files" FILES ${FirmwareUpdater_qt_UI_FILES})
source_group("Generated Files" FILES ${FirmwareUpdater_qt_QRC_GEN_SRCS}
                                    ${FirmwareUpdater_qt_UI_GEN_SRCS})

add_executable(FirmwareUpdater-qt WIN32 ${FirmwareUpdater_qt_SRCS}
                                    ${FirmwareUpdater_qt_HDRS}
                                    ${FirmwareUpdater_qt_QRC_GEN_SRCS}
                                    ${FirmwareUpdater_qt_UI_GEN_SRCS})
target_link_libraries(FirmwareUpdater-qt    YARP::YARP_OS
                                            YARP::YARP_name
                                            YARP::YARP_init
                                            YARP::YARP_math
                                            YARP::YARP_sig
                                            YARP::YARP_dev
                                            ACE
                                            ethLoaderLib
                                            canLoaderLib
                                            )
qticub_use_modules(FirmwareUpdater-qt   Core 
                                        Gui 
                                        Widgets)
qticub_deprecate_with_cmake_version(2.8.11)
if(WIN32 AND CMAKE_VERSION VERSION_LESS 2.8.11)
  target_link_libraries(FirmwareUpdater-qt Qt5::WinMain)
endif()

install(TARGETS FirmwareUpdater-qt COMPONENT utilities DESTINATION ${CMAKE_INSTALL_BINDIR})

if(NOT YARP_DEFAULT_GTK)
  set_target_properties(FirmwareUpdater-qt PROPERTIES OUTPUT_NAME FirmwareUpdater)
endif()

