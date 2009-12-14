
cmake_minimum_required(VERSION 2.4)

set(TARGET_NAME HandCtrl)
project(lib${TARGET_NAME})

find_package(ICUB)

find_package(YARP REQUIRED)
find_package(LIBVISLAB REQUIRED)
include_directories(${YARP_INCLUDE_DIRS} ${LIBVISLAB_INCLUDE_DIRS})

set(SOURCE_DIR "src/")

file(GLOB_RECURSE sources "${SOURCE_DIR}*.cpp")
file(GLOB_RECURSE headers "${SOURCE_DIR}*.h")
source_group("Source Files" FILES ${sources})
source_group("Header Files" FILES ${headers})

add_library(${TARGET_NAME} STATIC ${headers} ${sources})
target_link_libraries(${TARGET_NAME} ${YARP_LIBRARIES} ${LIBVISLAB_LIBRARIES})
install(TARGETS ${TARGET_NAME} RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
