# Copyright: (C) 2010 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(parametricCalibratorEth CATEGORY device TYPE yarp::dev::parametricCalibratorEth INCLUDE parametricCalibratorEth.h)

if(ENABLE_parametricCalibratorEth)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})

    yarp_add_plugin(parametricCalibratorEth parametricCalibratorEth.h parametricCalibratorEth.cpp)
    target_link_libraries(parametricCalibratorEth iCubDev)
    icub_export_plugin(parametricCalibratorEth)

  yarp_install(TARGETS parametricCalibratorEth
               COMPONENT Runtime
               LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
