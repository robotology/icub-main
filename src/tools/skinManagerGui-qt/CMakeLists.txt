# Copyright (C) 2014  iCub Facility, Istituto Italiano di Tecnologia
# Author: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

include(iCubUseQt5)

set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

include_directories(${skinDynLib_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

set(skinManagerGui_qt_SRCS main.cpp
                           loadingwidget.cpp
                           mainwindow.cpp
                           portthread.cpp
                           qcustomplot.cpp)

set(skinManagerGui_qt_HDRS loadingwidget.h
                           mainwindow.h
                           portthread.h
                           qcustomplot.h)

set(skinManagerGui_qt_QRC_FILES res.qrc)

set(skinManagerGui_qt_UI_FILES loadingwidget.ui
                               mainwindow.ui)

qt5_add_resources(skinManagerGui_qt_QRC_GEN_SRCS ${skinManagerGui_qt_QRC_FILES})
qt5_wrap_ui(skinManagerGui_qt_UI_GEN_SRCS ${skinManagerGui_qt_UI_FILES})

source_group("Source Files" FILES ${skinManagerGui_qt_SRCS})
source_group("Header Files" FILES ${skinManagerGui_qt_HDRS})
source_group("Resources Files" FILES ${skinManagerGui_qt_QRC_FILES})
source_group("UI Files" FILES ${skinManagerGui_qt_UI_FILES})
source_group("Generated Files" FILES ${skinManagerGui_qt_QRC_GEN_SRCS}
                                     ${skinManagerGui_qt_UI_GEN_SRCS})

add_executable(skinManagerGui-qt WIN32 ${skinManagerGui_qt_SRCS}
                                       ${skinManagerGui_qt_HDRS}
                                       ${skinManagerGui_qt_QRC_GEN_SRCS}
                                       ${skinManagerGui_qt_UI_GEN_SRCS})
target_link_libraries(skinManagerGui-qt YARP::YARP_OS
                                        YARP::YARP_init
                                        YARP::YARP_sig
                                        skinDynLib)

qticub_use_modules(skinManagerGui-qt Widgets
                                     PrintSupport)
qticub_deprecate_with_cmake_version(2.8.11)
if(WIN32 AND CMAKE_VERSION VERSION_LESS 2.8.11)
  target_link_libraries(skinManagerGui-qt Qt5::WinMain)
endif()

install(TARGETS skinManagerGui-qt COMPONENT utilities DESTINATION ${CMAKE_INSTALL_BINDIR})

if(NOT YARP_DEFAULT_GTK)
  set_target_properties(skinManagerGui-qt PROPERTIES OUTPUT_NAME skinManagerGui)
endif()
