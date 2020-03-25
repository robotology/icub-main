# Copyright: (C) 2010 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(embObjSkin CATEGORY device TYPE EmbObjSkin INCLUDE embObjSkin.h EXTRA_CONFIG WRAPPER=skinWrapper)

if (NOT SKIP_embObjSkin)

    set(ICUB_COMPILE_EMBOBJ_LIBRARY ON CACHE INTERNAL "use the embObjLib lib")

    include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                   ../skinLib)

    yarp_add_plugin(embObjSkin embObjSkin.h embObjSkin.cpp ../skinLib/SkinConfigReader.cpp ../skinLib/SkinDiagnostics.h)
    target_link_libraries(embObjSkin ethResources YARP::YARP_os icub_firmware_shared::canProtocolLib)
    icub_export_plugin(embObjSkin)
 
  yarp_install(TARGETS embObjSkin
               COMPONENT Runtime
               LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

endif (NOT SKIP_embObjSkin)
