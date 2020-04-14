# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(canBusSkin CATEGORY device TYPE CanBusSkin INCLUDE CanBusSkin.h EXTRA_CONFIG WRAPPER=skinwrapper)

if (NOT SKIP_canBusSkin)
if(ICUB_HAS_icub_firmware_shared)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                       ../skinLib/)

yarp_add_plugin(canBusSkin CanBusSkin.h CanBusSkin.cpp ../skinLib/SkinConfigReader.cpp ../skinLib/SkinDiagnostics.h)
target_link_libraries(canBusSkin YARP::YARP_os
                                 YARP::YARP_dev
                                 YARP::YARP_sig
                                 ${ICUB_LIBRARIES}
                                 icub_firmware_shared::canProtocolLib)

  yarp_install(TARGETS canBusSkin
               COMPONENT Runtime
               LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

else(ICUB_HAS_icub_firmware_shared)
  message(ERROR " canBusSkin: cannot find icub_firmware_shared library, turn off device
  embObj library can now be found in the icub-firmware-shared package that
  can be downloaded from here:
  .
       https://github.com/robotology/icub-firmware-shared
  .    ")
endif(ICUB_HAS_icub_firmware_shared)

endif (NOT SKIP_canBusSkin)
