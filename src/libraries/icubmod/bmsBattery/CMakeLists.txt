# Copyright: (C) 2015 iCub Facility
# Authors: Marco Randazzo <marco.randazzo@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(bmsBattery CATEGORY device TYPE BmsBattery INCLUDE bmsBattery.h EXTRA_CONFIG WRAPPER=BatteryWrapper)

if(NOT SKIP_bmsBattery)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                       ${YARP_INCLUDE_DIRS})

yarp_add_plugin(bmsBattery bmsBattery.h bmsBattery.cpp)
target_link_libraries(bmsBattery ${YARP_LIBRARIES})

icub_export_plugin(bmsBattery)

yarp_install(FILES bmsBattery.ini  DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

endif (NOT SKIP_bmsBattery)
