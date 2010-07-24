set(TARGET_NAME vislab)
project(lib${TARGET_NAME})

set(SOURCE_DIR "src/")

file(GLOB_RECURSE sources "${SOURCE_DIR}*.cpp")
file(GLOB_RECURSE headers "${SOURCE_DIR}*.h")
source_group("Source Files" FILES ${sources})
source_group("Header Files" FILES ${headers})

include_directories(${YARP_INCLUDE_DIRS})
add_library(${TARGET_NAME} ${headers} ${sources})
target_link_libraries(${TARGET_NAME} ${YARP_LIBRARIES})
install(TARGETS ${TARGET_NAME} DESTINATION lib)

