# Copyright: (C) 2013 iCub Facility
# Authors: Marco Randazzo <marco.randazzo@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PREPARE_DEVICE(canBusVirtualAnalogSensor TYPE CanBusVirtualAnalogSensor INCLUDE CanBusVirtualAnalogSensor.h WRAPPER grabber)

if (NOT SKIP_canBusVirtualAnalogSensor)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                       ${YARP_INCLUDE_DIRS})

yarp_add_plugin(canBusVirtualAnalogSensor CanBusVirtualAnalogSensor.h CanBusVirtualAnalogSensor.cpp)
target_link_libraries(canBusVirtualAnalogSensor ${YARP_LIBRARIES})

icub_export_library(canBusVirtualAnalogSensor)

endif (NOT SKIP_canBusVirtualAnalogSensor)
