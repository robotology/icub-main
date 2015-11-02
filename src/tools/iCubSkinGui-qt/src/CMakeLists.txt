# Copyright (C) 2015  iCub Facility, Istituto Italiano di Tecnologia
# Author: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT


# Remove -rdynamic from CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS.
# This is needed for some weird reason, or executables will not load resources
# from the plugins.
string(REPLACE "-rdynamic" "" CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS}")


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/qtquick2applicationviewer)

set(iCubSkinGui_qt_SRCS main.cpp)
set(iCubSkinGui_qt_HDRS )
set(iCubSkinGui_qt_QRC_FILES qml.qrc)
set(iCubSkinGui_qt_QML_FILES main.qml)

qt5_add_resources(iCubSkinGui_qt_QRC_GEN_SRCS ${iCubSkinGui_qt_QRC_FILES})

source_group("Source Files" FILES ${iCubSkinGui_qt_SRCS})
source_group("Header Files" FILES ${iCubSkinGui_qt_HDRS})
source_group("Resources Files" FILES ${iCubSkinGui_qt_QRC_FILES})
source_group("QML Files" FILES ${iCubSkinGui_qt_QML_FILES})
source_group("Generated Files" FILES ${iCubSkinGui_qt_QRC_GEN_SRCS})

qticub_use_qml_plugin()

add_executable(iCubSkinGui-qt WIN32 ${iCubSkinGui_qt_SRCS}
                                    ${iCubSkinGui_qt_HDRS}
                                    ${iCubSkinGui_qt_QRC_GEN_SRCS})
qticub_use_modules(iCubSkinGui-qt Widgets
                                  Qml
                                  Quick)
qticub_deprecate_with_cmake_version(2.8.11)
if(WIN32 AND CMAKE_VERSION VERSION_LESS 2.8.11)
  target_link_libraries(iCubSkinGui-qt Qt5::WinMain)
endif()

install(TARGETS iCubSkinGui-qt
        COMPONENT utilities
        DESTINATION ${CMAKE_INSTALL_BINDIR})

if(NOT YARP_DEFAULT_GTK)
  set_target_properties(iCubSkinGui-qt PROPERTIES OUTPUT_NAME iCubSkinGui)
endif()
