cmake_minimum_required(VERSION 2.6)

set(PROJECTNAME boostExample)

project(${PROJECTNAME})

find_package(YARP)
find_package(ICUB)

SET(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})


include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})
include_directories(${boostMIL_INCLUDE_DIRS})


INCLUDE(iCubOptions)
INCLUDE(iCubHelpers)

SET(folder_source main.cpp)

SOURCE_GROUP("Source Files" FILES ${folder_source})

add_executable(${PROJECTNAME} ${folder_source})

target_link_libraries(${PROJECTNAME} boostMIL ${ICUB_LIBRARIES} ${YARP_LIBRARIES})