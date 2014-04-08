# Copyright: (C) 2012 RobotCub Consortium
# Authors: Lorenzo Natale, Alessandro Scalzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PREPARE_DEVICE(sharedcan TYPE CanBusAccessPoint
    INCLUDE SharedCanBus.h)


IF (NOT SKIP_sharedcan)
    include_directories(${YARP_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../motionControlLib/)
    yarp_add_plugin(shcan SharedCanBus.h SharedCanBus.cpp)
    target_link_libraries(shcan ${YARP_LIBRARIES})
    
    icub_export_library(shcan)
ENDIF (NOT SKIP_sharedcan)
