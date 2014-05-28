# Copyright: (C) 2013 iCub Facility
# Authors: Marco Randazzo and Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PREPARE_DEVICE(canBusInertialMTB TYPE CanBusInertialMTB INCLUDE CanBusInertialMTB.h WRAPPER AnalogSensor)

if (NOT SKIP_canBusInertialMTB)

if(ICUB_HAS_icub_firmware_shared)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                       ${YARP_INCLUDE_DIRS})

yarp_add_plugin(canBusInertialMTB CanBusInertialMTB.h CanBusInertialMTB.cpp)
target_link_libraries(canBusInertialMTB ${YARP_LIBRARIES})

icub_export_library(canBusInertialMTB)

else(ICUB_HAS_icub_firmware_shared)
  message(ERROR " canBusInertialMTB: cannot find icub_firmware_shared library, turn off device
  embObj library can now be found in the icub-firmware-shared package that
  can be downloaded from here:
  .
       https://github.com/robotology/icub-firmware-shared
  .    ")
endif(ICUB_HAS_icub_firmware_shared)

endif (NOT SKIP_canBusInertialMTB)
