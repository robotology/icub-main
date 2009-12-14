
cmake_minimum_required(VERSION 2.4)

set(TARGET_NAME eyecub)
project(lib${TARGET_NAME})

find_package(ICUB)

find_package(YARP REQUIRED)
find_package(IPOPT REQUIRED)
find_package(CTRLLIB REQUIRED)
find_package(IKIN REQUIRED)
find_package(LIBVISLAB REQUIRED)
include_directories(${LIBEYECUB_INCLUDE_DIRS} ${IKIN_INCLUDE_DIRS} ${CTRLLIB_INCLUDE_DIRS} ${LIBVISLAB_INCLUDE_DIRS})

set(SOURCE_DIR "src/")

file(GLOB_RECURSE sources "${SOURCE_DIR}*.cpp")
file(GLOB_RECURSE headers "${SOURCE_DIR}*.h")
source_group("Source Files" FILES ${sources})
source_group("Header Files" FILES ${headers})

add_library(${TARGET_NAME} ${headers} ${sources})
target_link_libraries(${TARGET_NAME} ${IKIN_LIBRARIES} ${CTRLLIB_LIBRARIES} ${YARP_LIBRARIES} ${LIBVISLAB_LIBRARIES})
install(TARGETS ${TARGET_NAME} RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
