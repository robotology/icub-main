# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PREPARE_DEVICE(canBusAnalogSensor TYPE CanBusAnalogSensor INCLUDE CanBusAnalogSensor.h WRAPPER grabber)

if (NOT SKIP_canBusAnalogSensor)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                       ${YARP_INCLUDE_DIRS})

add_library(canBusAnalogSensor CanBusAnalogSensor.h CanBusAnalogSensor.cpp)
target_link_libraries(canBusAnalogSensor ${YARP_LIBRARIES})

icub_export_library(canBusAnalogSensor)

endif (NOT SKIP_canBusAnalogSensor)
