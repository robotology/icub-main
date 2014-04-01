# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PREPARE_DEVICE(canBusSkin TYPE CanBusSkin INCLUDE CanBusSkin.h WRAPPER grabber)

if (NOT SKIP_canBusSkin)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                       ${YARP_INCLUDE_DIRS})

yarp_add_plugin(canBusSkin CanBusSkin.h CanBusSkin.cpp)
target_link_libraries(canBusSkin ${YARP_LIBRARIES})

icub_export_library(canBusSkin)

endif (NOT SKIP_canBusSkin)
