
cmake_minimum_required(VERSION 2.4)

set(PROJECTNAME abstractHandCtrl)
project(${PROJECTNAME})

find_package(ICUB)

find_package(YARP REQUIRED)
find_package(IPOPT REQUIRED)
find_package(LIBHANDCTRL REQUIRED)
find_package(LIBVISLAB REQUIRED)
include_directories(${YARP_INCLUDE_DIRS} ${LIBVISLAB_INCLUDE_DIRS} ${LIBHANDCTRL_INCLUDE_DIRS})

set(SOURCE_DIR "src/")

file(GLOB_RECURSE sources "${SOURCE_DIR}*.cpp")
file(GLOB_RECURSE headers "${SOURCE_DIR}*.h")
source_group("Source Files" FILES ${sources})
source_group("Header Files" FILES ${headers})

add_executable(${PROJECTNAME} ${headers} ${sources})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${LIBVISLAB_LIBRARIES} ${LIBHANDCTRL_LIBRARIES})
install(TARGETS ${PROJECTNAME} RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
