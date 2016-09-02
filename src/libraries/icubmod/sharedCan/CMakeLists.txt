# Copyright: (C) 2012 RobotCub Consortium
# Authors: Lorenzo Natale, Alessandro Scalzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_device(sharedcan TYPE yarp::dev::CanBusAccessPoint
    INCLUDE SharedCanBus.h)


IF (NOT SKIP_sharedcan)
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
    include_directories(${YARP_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../motionControlLib/)
    yarp_add_plugin(shcan SharedCanBus.h SharedCanBus.cpp)
    target_link_libraries(shcan ${YARP_LIBRARIES})
    
    icub_export_plugin(shcan)
    yarp_install(FILES sharedcan.ini  DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})
ENDIF (NOT SKIP_sharedcan)
