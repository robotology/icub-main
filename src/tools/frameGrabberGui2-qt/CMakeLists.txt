# Copyright (C) 2014  iCub Facility, Istituto Italiano di Tecnologia
# Author: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

include(iCubUseQt5)

set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

include_directories(${YARP_INCLUDE_DIRS})

set(frameGrabberGui2_qt_SRCS dc1394slider.cpp
                              dc1394sliderwb.cpp
                              dc1394thread.cpp
                              loadingwidget.cpp
                              main.cpp
                              mainwindow.cpp)
set(frameGrabberGui2_qt_HDRS dc1394SliderBase.h
                              dc1394slider.h
                              dc1394sliderwb.h
                              dc1394thread.h
                              loadingwidget.h
                              log.h
                              mainwindow.h)
set(frameGrabberGui2_qt_QRC_FILES res.qrc)
set(frameGrabberGui2_qt_UI_FILES dc1394slider.ui
                                 dc1394sliderwb.ui
                                 loadingwidget.ui
                                 mainwindow.ui)

qt5_add_resources(frameGrabberGui2_qt_QRC_GEN_SRCS ${frameGrabberGui2_qt_QRC_FILES})
qt5_wrap_ui(frameGrabberGui2_qt_UI_GEN_SRCS ${frameGrabberGui2_qt_UI_FILES})

source_group("Source Files" FILES ${frameGrabberGui2_qt_SRCS})
source_group("Header Files" FILES ${frameGrabberGui2_qt_HDRS})
source_group("Resources Files" FILES ${frameGrabberGui2_qt_QRC_FILES})
source_group("UI Files" FILES ${frameGrabberGui2_qt_UI_FILES})
source_group("Generated Files" FILES ${frameGrabberGui2_qt_QRC_GEN_SRCS}
                                    ${frameGrabberGui2_qt_UI_GEN_SRCS})

add_executable(frameGrabberGui2-qt WIN32 ${frameGrabberGui2_qt_SRCS}
                                    ${frameGrabberGui2_qt_HDRS}
                                    ${frameGrabberGui2_qt_QRC_GEN_SRCS}
                                    ${frameGrabberGui2_qt_UI_GEN_SRCS})
target_link_libraries(frameGrabberGui2-qt YARP::YARP_OS
                                          YARP::YARP_init
                                          YARP::YARP_dev)
qticub_use_modules(frameGrabberGui2-qt Widgets
                                       Concurrent)
qticub_deprecate_with_cmake_version(2.8.11)
if(WIN32 AND CMAKE_VERSION VERSION_LESS 2.8.11)
  target_link_libraries(frameGrabberGui2-qt Qt5::WinMain)
endif()

install(TARGETS frameGrabberGui2-qt COMPONENT utilities DESTINATION ${CMAKE_INSTALL_BINDIR})

if(NOT YARP_DEFAULT_GTK)
  set_target_properties(frameGrabberGui2-qt PROPERTIES OUTPUT_NAME frameGrabberGui2)
endif()
