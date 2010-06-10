cmake_minimum_required(VERSION 2.6)

project(salience)

set(CMAKE_MODULE_PATH ${YARP_DIR}/conf ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${ICUB_DIR}/conf ${CMAKE_MODULE_PATH})

#find_package(iCubVis)

file(GLOB folder_source src/*.cpp)
file(GLOB folder_header include/iCub/*.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${PROJECT_SOURCE_DIR}/include ${iCubVis_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(salience ${folder_source} ${folder_header})
target_link_libraries(salience iCubVis ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS salience DESTINATION bin)
