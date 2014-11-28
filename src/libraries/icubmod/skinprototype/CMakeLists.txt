# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PREPARE_DEVICE(skinprototype TYPE SkinPrototype INCLUDE skinprototype.h WRAPPER skinWrapper)

if (NOT SKIP_skinprototype)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                       ${YARP_INCLUDE_DIRS})

yarp_add_plugin(skinprototype skinprototype.h skinprototype.cpp)
target_link_libraries(skinprototype ${YARP_LIBRARIES})

icub_export_library(skinprototype)

endif (NOT SKIP_skinprototype)
