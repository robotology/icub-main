PREPARE_DEVICE(skinprototype TYPE SkinPrototype INCLUDE skinprototype.h WRAPPER grabber)

if (NOT SKIP_static_grabber)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                       ${YARP_INCLUDE_DIRS})

add_library(skinprototype skinprototype.h skinprototype.cpp)
target_link_libraries(skinprototype ${YARP_LIBRARIES})

icub_export_library(skinprototype "" include/iCub "")

endif (NOT SKIP_static_grabber)
