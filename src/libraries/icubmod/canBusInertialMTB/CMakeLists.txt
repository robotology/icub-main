# Copyright: (C) 2013 iCub Facility
# Authors: Marco Randazzo and Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PREPARE_DEVICE(canBusInertialMTB TYPE CanBusInertialMTB INCLUDE CanBusInertialMTB.h WRAPPER AnalogSensor)

if (NOT SKIP_canBusInertialMTB)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                       ${YARP_INCLUDE_DIRS})

yarp_add_plugin(canBusInertialMTB CanBusInertialMTB.h CanBusInertialMTB.cpp)
target_link_libraries(canBusInertialMTB ${YARP_LIBRARIES})

icub_export_library(canBusInertialMTB)

endif (NOT SKIP_canBusInertialMTB)
