# Copyright: (C) 2010 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


yarp_prepare_plugin(parametricCalibrator CATEGORY device TYPE yarp::dev::parametricCalibrator INCLUDE parametricCalibrator.h)

if(ENABLE_parametricCalibrator)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})

    yarp_add_plugin(parametricCalibrator parametricCalibrator.h parametricCalibrator.cpp)
    target_link_libraries(parametricCalibrator iCubDev)
    icub_export_plugin(parametricCalibrator)
    
  yarp_install(TARGETS parametricCalibrator
               COMPONENT Runtime
               LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
