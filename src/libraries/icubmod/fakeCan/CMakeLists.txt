# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(fakecan CATEGORY device
                            TYPE yarp::dev::FakeCan
                            INCLUDE fakeCan.h)


IF (NOT SKIP_fakecan)
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
    include_directories(${YARP_INCLUDE_DIRS})
    yarp_add_plugin(fcan fakeCan.cpp fakeBoard.cpp fakeCan.h fakeBoard.h fbCanBusMessage.h msgList.h)
    target_link_libraries(fcan ${YARP_LIBRARIES})
    
    icub_export_plugin(fcan)
    yarp_install(FILES fakecan.ini  DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})
ENDIF (NOT SKIP_fakecan)
