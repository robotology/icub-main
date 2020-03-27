# Copyright (C) 2014  iCub Facility, Istituto Italiano di Tecnologia
# Author: Daniele E. Domenichelli <daniele.domenichelli@iit.it>, Marco Acccame <marco.accame@iit.it>
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT


if(NOT ICUB_USE_icub_firmware_shared)
  message(STATUS "icub-firmware-shared not selected, disabling FirmwareUpdater")
  return()
endif()

include(iCubUseQt5)

set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})


set(FirmwareUpdater_qt_SRCS changeaddressdialog.cpp
                            changeinfodialog.cpp
                            firmwareupdatercore.cpp
                            main.cpp
                            mainwindow.cpp
                            calibrationwindow.cpp
                            customtreewidgetitem.cpp
                            customspinbox.cpp
                            customcombobox.cpp
                            selectioncheckbox.cpp
                            straincalibgui.cpp)
set(FirmwareUpdater_qt_HDRS changeaddressdialog.h
                            changeinfodialog.h
                            firmwareupdatercore.h
                            mainwindow.h
                            calibrationwindow.h
                            customtreewidgetitem.h
                            customspinbox.h
                            customcombobox.h
                            selectioncheckbox.h
                            straincalibgui.h)
set(FirmwareUpdater_qt_QRC_FILES res.qrc)
set(FirmwareUpdater_qt_UI_FILES changeaddressdialog.ui
                            changeinfodialog.ui
                            calibrationwindow.ui
                            mainwindow.ui
                            straincalibgui.ui)

qt5_add_resources(FirmwareUpdater_qt_QRC_GEN_SRCS ${FirmwareUpdater_qt_QRC_FILES})
qt5_wrap_ui(FirmwareUpdater_qt_UI_GEN_SRCS ${FirmwareUpdater_qt_UI_FILES})

source_group("Source Files" FILES ${FirmwareUpdater_qt_SRCS})
source_group("Header Files" FILES ${FirmwareUpdater_qt_HDRS})
source_group("Resources Files" FILES ${FirmwareUpdater_qt_QRC_FILES})
source_group("UI Files" FILES ${FirmwareUpdater_qt_UI_FILES})
source_group("Generated Files" FILES ${FirmwareUpdater_qt_QRC_GEN_SRCS}
                                    ${FirmwareUpdater_qt_UI_GEN_SRCS})

add_executable(FirmwareUpdater-qt WIN32 ${FirmwareUpdater_qt_SRCS}
                                        ${CMAKE_SOURCE_DIR}/src/libraries/icubmod/embObjLib/FeatureInterface.cpp
                                        ${FirmwareUpdater_qt_HDRS}
                                        ${FirmwareUpdater_qt_QRC_GEN_SRCS}
                                        ${FirmwareUpdater_qt_UI_GEN_SRCS})
target_link_libraries(FirmwareUpdater-qt    YARP::YARP_init
                                            YARP::YARP_math
                                            YARP::YARP_sig
                                            YARP::YARP_dev
                                            ethLoaderLib
                                            canLoaderLib
                                            strainCalibLib
                                            )
qticub_use_modules(FirmwareUpdater-qt   Core 
                                        Gui 
                                        Widgets)
if(WIN32 AND CMAKE_VERSION VERSION_LESS 2.8.11)
  target_link_libraries(FirmwareUpdater-qt Qt5::WinMain)
endif()

install(TARGETS FirmwareUpdater-qt COMPONENT utilities DESTINATION ${CMAKE_INSTALL_BINDIR})

if(NOT YARP_DEFAULT_GTK)
  set_target_properties(FirmwareUpdater-qt PROPERTIES OUTPUT_NAME FirmwareUpdater)
endif()

