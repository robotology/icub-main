
cmake_minimum_required(VERSION 2.4)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(TARGET_NAME vislab_YARP)
project(lib${TARGET_NAME})

find_package(ICUB)

find_package(YARP REQUIRED)
find_package(LIBVISLAB_BASE REQUIRED)
include_directories(${YARP_INCLUDE_DIRS} ${LIBVISLAB_BASE_INCLUDE_DIRS})

set(SOURCE_DIR "src/")

file(GLOB_RECURSE sources "${SOURCE_DIR}*.cpp")
file(GLOB_RECURSE headers "${SOURCE_DIR}*.h")
source_group("Source Files" FILES ${sources})
source_group("Header Files" FILES ${headers})

add_library(${TARGET_NAME} STATIC ${headers} ${sources})
target_link_libraries(${TARGET_NAME} ${YARP_LIBRARIES} ${LIBVISLAB_BASE_LIBRARIES})

