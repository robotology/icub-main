
cmake_minimum_required(VERSION 2.4)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

SET(PROJECTNAME gloveCtrl)
PROJECT(${PROJECTNAME})

find_package(ICUB)

find_package(YARP REQUIRED)
find_package(LIBHANDCTRL REQUIRED)
find_package(LIBVISLAB REQUIRED)

find_package(libglove REQUIRED)
find_package(FlockOfBirds)

include_directories(${YARP_INCLUDE_DIRS} ${LIBVISLAB_INCLUDE_DIRS} ${LIBHANDCTRL_INCLUDE_DIRS} ${LIBGLOVE_INCLUDE_DIRS} ${FLOCKOFBIRDS_INCLUDE_DIRS})

set(SOURCE_DIR "src/")

set(USE_FLOCKOFBIRDS ${FLOCKOFBORDS_FOUND})
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE_DIR}iCub/vislab/glovectrl/config.h.in ${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE_DIR}iCub/vislab/glovectrl/config.h)

file(GLOB_RECURSE sources "${SOURCE_DIR}*.cpp")
file(GLOB_RECURSE headers "${SOURCE_DIR}*.h")
source_group("Source Files" FILES ${sources})
source_group("Header Files" FILES ${headers})

add_executable(${PROJECTNAME} ${headers} ${sources})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${LIBVISLAB_LIBRARIES} ${LIBHANDCTRL_LIBRARIES}  ${LIBGLOVE_LIBRARIES} ${FLOCKOFBIRDS_LIBRARIES})
install(TARGETS ${PROJECTNAME} RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
