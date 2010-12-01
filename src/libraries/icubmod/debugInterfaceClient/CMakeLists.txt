PREPARE_DEVICE(debugInterfaceClient TYPE DebugInterfaceClient INCLUDE debugInterfaceClient.h WRAPPER controlboard)

if (NOT SKIP_debugInterfaceClient)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
			${iCubDev_INCLUDE_DIRS}
                       ${YARP_INCLUDE_DIRS})

add_library(debugInterfaceClient debugInterfaceClient.h debugInterfaceClient.cpp)
target_link_libraries(debugInterfaceClient ${YARP_LIBRARIES})

icub_export_library(debugInterfaceClient)

endif (NOT SKIP_debugInterfaceClient)
