# Copyright (C) 2014  iCub Facility, Istituto Italiano di Tecnologia
# Author: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

include(iCubUseQt5)

set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

set(skinManagerGui_SRCS    main.cpp
                           loadingwidget.cpp
                           mainwindow.cpp
                           portthread.cpp
                           qcustomplot.cpp)

set(skinManagerGui_HDRS    loadingwidget.h
                           mainwindow.h
                           portthread.h
                           qcustomplot.h)

set(skinManagerGui_QRC_FILES res.qrc)

set(skinManagerGui_UI_FILES    loadingwidget.ui
                               mainwindow.ui)

qt5_add_resources(skinManagerGui_QRC_GEN_SRCS ${skinManagerGui_QRC_FILES})
qt5_wrap_ui(skinManagerGui_UI_GEN_SRCS ${skinManagerGui_UI_FILES})

source_group("Source Files" FILES ${skinManagerGui_SRCS})
source_group("Header Files" FILES ${skinManagerGui_HDRS})
source_group("Resources Files" FILES ${skinManagerGui_QRC_FILES})
source_group("UI Files" FILES ${skinManagerGui_UI_FILES})
source_group("Generated Files" FILES ${skinManagerGui_QRC_GEN_SRCS}
                                     ${skinManagerGui_UI_GEN_SRCS})

add_executable(skinManagerGui WIN32    ${skinManagerGui_SRCS}
                                       ${skinManagerGui_HDRS}
                                       ${skinManagerGui_QRC_GEN_SRCS}
                                       ${skinManagerGui_UI_GEN_SRCS})
target_link_libraries(skinManagerGui    YARP::YARP_OS
                                        YARP::YARP_init
                                        YARP::YARP_sig
                                        skinDynLib)

qticub_use_modules(skinManagerGui Widgets
                                     PrintSupport)
if(WIN32 AND CMAKE_VERSION VERSION_LESS 2.8.11)
  target_link_libraries(skinManagerGui Qt5::WinMain)
endif()

install(TARGETS skinManagerGui COMPONENT utilities DESTINATION ${CMAKE_INSTALL_BINDIR})
