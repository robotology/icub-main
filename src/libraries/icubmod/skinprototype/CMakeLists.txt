# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(skinprototype CATEGORY device TYPE SkinPrototype INCLUDE skinprototype.h EXTRA_CONFIG WRAPPER=skinWrapper)

if (NOT SKIP_skinprototype)

    include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS})

    yarp_add_plugin(skinprototype skinprototype.h skinprototype.cpp)
    target_link_libraries(skinprototype ${YARP_LIBRARIES})

    icub_export_plugin(skinprototype)
    yarp_install(FILES skinprototype.ini  DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

endif (NOT SKIP_skinprototype)
