# Copyright: (C) 2013 iCub Facility
# Authors: Marco Randazzo <marco.randazzo@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(canBusVirtualAnalogSensor CATEGORY device TYPE CanBusVirtualAnalogSensor INCLUDE CanBusVirtualAnalogSensor.h EXTRA_CONFIG WRAPPER=virtualAnalogServer)

if (NOT SKIP_canBusVirtualAnalogSensor)

if(ICUB_HAS_icub_firmware_shared)

    include_directories(${CMAKE_CURRENT_SOURCE_DIR})

    yarp_add_plugin(canBusVirtualAnalogSensor CanBusVirtualAnalogSensor.h CanBusVirtualAnalogSensor.cpp)
    target_link_libraries(canBusVirtualAnalogSensor ${YARP_LIBRARIES})

    icub_export_plugin(canBusVirtualAnalogSensor)

          yarp_install(TARGETS canBusVirtualAnalogSensor
               COMPONENT Runtime
               LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})


else(ICUB_HAS_icub_firmware_shared)
  message(ERROR " canBusVirtualAnalogSensor: cannot find icub_firmware_shared library, turn off device
  embObj library can now be found in the icub-firmware-shared package that
  can be downloaded from here:
  .
       https://github.com/robotology/icub-firmware-shared
  .    ")
endif(ICUB_HAS_icub_firmware_shared)

endif (NOT SKIP_canBusVirtualAnalogSensor)
