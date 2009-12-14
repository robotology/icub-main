
cmake_minimum_required(VERSION 2.4)

SET(PROJECTNAME eye2world)
PROJECT(${PROJECTNAME})

find_package(ICUB)

find_package(YARP REQUIRED)
find_package(IPOPT REQUIRED)
find_package(CTRLLIB REQUIRED)
find_package(IKIN REQUIRED)
find_package(LIBEYECUB REQUIRED)
find_package(LIBVISLAB REQUIRED)
include_directories(${LIBEYECUB_INCLUDE_DIRS} ${IKIN_INCLUDE_DIRS} ${CTRLLIB_INCLUDE_DIRS} ${LIBVISLAB_INCLUDE_DIRS})

set(SOURCE_DIR "src/")

file(GLOB_RECURSE sources "${SOURCE_DIR}*.cpp")
file(GLOB_RECURSE headers "${SOURCE_DIR}*.h")
source_group("Source Files" FILES ${sources})
source_group("Header Files" FILES ${headers})

add_executable(${PROJECTNAME} ${headers} ${sources})
target_link_libraries(${PROJECTNAME} ${LIBEYECUB_LIBRARIES} ${IKIN_LIBRARIES} ${CTRLLIB_LIBRARIES} ${LIBVISLAB_LIBRARIES})
install(TARGETS ${PROJECTNAME} RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
