# Copyright: (C) 2015 RobotCub Consortium
# Authors: Marco Randazzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(canLoader-console)

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../canLoaderLib)

add_executable(${PROJECT_NAME} ${folder_header} ${folder_source} ${CMAKE_SOURCE_DIR}/src/libraries/icubmod/embObjLib/FeatureInterface.cpp )

target_link_libraries(${PROJECT_NAME} canLoaderLib)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

