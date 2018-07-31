# Copyright: (C) 2013 iCub Facility
# Authors: Marco Randazzo <marco.randazzo@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(canBusAnalogSensor CATEGORY device TYPE CanBusAnalogSensor INCLUDE CanBusAnalogSensor.h EXTRA_CONFIG WRAPPER=AnalogServer)

if(NOT SKIP_canBusAnalogSensor)

if(ICUB_HAS_icub_firmware_shared)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

yarp_add_plugin(canBusAnalogSensor CanBusAnalogSensor.h CanBusAnalogSensor.cpp)
target_link_libraries(canBusAnalogSensor ${YARP_LIBRARIES})

icub_export_plugin(canBusAnalogSensor)

yarp_install(FILES canBusAnalogSensor.ini  DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

else(ICUB_HAS_icub_firmware_shared)
  message(ERROR " canBusAnalogSensor: cannot find icub_firmware_shared library, turn off device
  embObj library can now be found in the icub-firmware-shared package that
  can be downloaded from here:
  .
       https://github.com/robotology/icub-firmware-shared
  .    ")
endif(ICUB_HAS_icub_firmware_shared)

endif (NOT SKIP_canBusAnalogSensor)
