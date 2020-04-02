# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale, Alessandro Scalzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME ethLoader)
project(ethLoader)

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../ethLoaderLib)
include_directories(${GTK2_INCLUDE_DIRS})



add_executable(${PROJECT_NAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECT_NAME}
                      ethLoaderLib 
                      ${GTK2_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

