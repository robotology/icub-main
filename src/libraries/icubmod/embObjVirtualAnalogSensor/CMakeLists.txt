# Copyright: (C) 2012 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(embObjVirtualAnalogSensor CATEGORY device TYPE yarp::dev::embObjVirtualAnalogSensor INCLUDE embObjVirtualAnalogSensor.h EXTRA_CONFIG WRAPPER=virtualAnalogServer)

IF (NOT SKIP_embObjVirtualAnalogSensor)

    set(ICUB_COMPILE_EMBOBJ_LIBRARY ON CACHE INTERNAL "use the embObjLib lib")
    INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

    yarp_add_plugin(embObjVirtualAnalogSensor embObjVirtualAnalogSensor.cpp embObjVirtualAnalogSensor.h)

    TARGET_LINK_LIBRARIES(embObjVirtualAnalogSensor ethResources iCubDev)
    icub_export_plugin(embObjVirtualAnalogSensor)

  yarp_install(TARGETS embObjVirtualAnalogSensor
               COMPONENT Runtime
               LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

ENDIF ()

