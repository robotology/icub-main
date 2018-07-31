# Copyright: (C) 2010 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(parametricCalibratorEth CATEGORY device TYPE yarp::dev::parametricCalibratorEth INCLUDE parametricCalibratorEth.h)

if(ENABLE_parametricCalibratorEth)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                       ${iCubDev_INCLUDE_DIRS})

    yarp_add_plugin(parametricCalibratorEth parametricCalibratorEth.h parametricCalibratorEth.cpp)
    target_link_libraries(parametricCalibratorEth ${YARP_LIBRARIES})
    icub_export_plugin(parametricCalibratorEth)
    yarp_install(FILES parametricCalibratorEth.ini  DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})
endif(ENABLE_parametricCalibratorEth)
