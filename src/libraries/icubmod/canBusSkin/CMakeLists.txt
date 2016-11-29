# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(canBusSkin CATEGORY device TYPE CanBusSkin INCLUDE CanBusSkin.h EXTRA_CONFIG WRAPPER=grabber)

if (NOT SKIP_canBusSkin)
if(ICUB_HAS_icub_firmware_shared)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                       ${YARP_INCLUDE_DIRS} 
                       ../skinLib/
                       ${icub_firmware_shared_canProtocolLib_INCLUDE_DIR})

yarp_add_plugin(canBusSkin CanBusSkin.h CanBusSkin.cpp ../skinLib/SkinConfigReader.cpp ../skinLib/SkinDiagnostics.h)
target_link_libraries(canBusSkin ${YARP_LIBRARIES} ${ICUB_LIBRARIES})

icub_export_plugin(canBusSkin)
yarp_install(FILES canBusSkin.ini  DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

else(ICUB_HAS_icub_firmware_shared)
  message(ERROR " canBusSkin: cannot find icub_firmware_shared library, turn off device
  embObj library can now be found in the icub-firmware-shared package that
  can be downloaded from here:
  .
       https://github.com/robotology/icub-firmware-shared
  .    ")
endif(ICUB_HAS_icub_firmware_shared)

endif (NOT SKIP_canBusSkin)
