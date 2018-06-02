# Copyright (C) 2015  iCub Facility, Istituto Italiano di Tecnologia
# Author: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 

set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

include_directories(${GSL_INCLUDE_DIRS})

set(QtICubSkinGuiPlugin_SRCS Fingertip2Left.cpp
                             Fingertip2Right.cpp
                             Fingertip.cpp
                             PalmLeft.cpp
                             PalmRight.cpp
                             qticubskinguiplugin.cpp
                             qticubskinguiplugin_plugin.cpp
                             Quad16.cpp
                             SkinMeshThreadCan.cpp
                             SkinMeshThreadPort.cpp
                             TouchSensor.cpp
                             Triangle_10pad.cpp
                             Triangle.cpp
							 CER_SH_PDL.cpp)

set(QtICubSkinGuiPlugin_HDRS qticubskinguiplugin.h
                             qticubskinguiplugin_plugin.h
                             include/Fingertip2Left.h
                             include/Fingertip2Right.h
                             include/Fingertip.h
                             include/PalmLeft.h
                             include/PalmRight.h
                             include/Quad16.h
                             include/SkinMeshThreadCan.h
                             include/SkinMeshThreadPort.h
                             include/TouchSensor.h
                             include/Triangle_10pad.h
                             include/Triangle.h
							 include/CER_SH_PDL.h)

source_group("Source Files" FILES ${QtICubSkinGuiPlugin_SRCS})
source_group("Header Files" FILES ${QtICubSkinGuiPlugin_HDRS})

set(QtICubSkinGuiPlugin_QMLDIR "robotology/icub/skingui")

add_library(QtICubSkinGuiPlugin MODULE ${QtICubSkinGuiPlugin_SRCS}
                                       ${QtICubSkinGuiPlugin_HDRS})
target_link_libraries(QtICubSkinGuiPlugin YARP::YARP_OS
                                          YARP::YARP_init
                                          YARP::YARP_sig
                                          YARP::YARP_dev
                                          ${GSL_LIBRARIES})
qticub_use_modules(QtICubSkinGuiPlugin Quick)

qticub_qml_plugin(QtICubSkinGuiPlugin "${QtICubSkinGuiPlugin_QMLDIR}")

install(TARGETS QtICubSkinGuiPlugin
        COMPONENT utilities
        RUNTIME DESTINATION "${CMAKE_INSTALL_QMLDIR}/${QtICubSkinGuiPlugin_QMLDIR}"
        LIBRARY DESTINATION "${CMAKE_INSTALL_QMLDIR}/${QtICubSkinGuiPlugin_QMLDIR}"
        ARCHIVE DESTINATION "${CMAKE_INSTALL_QMLDIR}/${QtICubSkinGuiPlugin_QMLDIR}")

install(FILES qmldir DESTINATION "${CMAKE_INSTALL_QMLDIR}/${QtICubSkinGuiPlugin_QMLDIR}")
