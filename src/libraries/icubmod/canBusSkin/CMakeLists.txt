# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PREPARE_DEVICE(canBusSkin TYPE CanBusSkin INCLUDE CanBusSkin.h WRAPPER grabber)

if (NOT SKIP_canBusSkin)

if(ICUB_HAS_icub_firmware_shared)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                       ${YARP_INCLUDE_DIRS})

yarp_add_plugin(canBusSkin CanBusSkin.h CanBusSkin.cpp)
target_link_libraries(canBusSkin ${YARP_LIBRARIES})

icub_export_library(canBusSkin)


else(ICUB_HAS_icub_firmware_shared)
  message(ERROR " canBusSkin: cannot find icub_firmware_shared library, turn off device
  embObj library can now be found in the icub-firmware-shared package that
  can be downloaded from here:
  .
       https://github.com/robotology/icub-firmware-shared
  .    ")
endif(ICUB_HAS_icub_firmware_shared)

endif (NOT SKIP_canBusSkin)
