# Copyright: (C) 2012 RobotCub Consortium
# Authors: Valentina Gaggero
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(embObjIMU    CATEGORY device
                                 TYPE yarp::dev::embObjIMU
                                 INCLUDE embObjIMU.h
                                 EXTRA_CONFIG WRAPPER=inertial)

IF (NOT SKIP_embObjIMU)


  set(ICUB_COMPILE_EMBOBJ_LIBRARY ON CACHE INTERNAL "use the embObjLib lib")

#   message(INFO " embObjIMU - embObj_includes: ${embObj_includes}, ${CMAKE_CURRENT_SOURCE_DIR}/")
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${iCubDev_INCLUDE_DIRS} )

  yarp_add_plugin(embObjIMU embObjIMU.cpp embObjIMU.h eo_imu_privData.h  eo_imu_privData.cpp imuMeasureConverter.cpp imuMeasureConverter.h )
  TARGET_LINK_LIBRARIES(embObjIMU ethResources ${YARP_LIBRARIES} ${ACE_LIBRARIES})
  icub_export_plugin(embObjIMU)

        yarp_install(TARGETS embObjIMU
               COMPONENT Runtime
               LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

ENDIF (NOT SKIP_embObjIMU)

