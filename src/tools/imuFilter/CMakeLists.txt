# Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME imuFilter)
project(${PROJECTNAME})

file(GLOB source_files main.cpp)
set(doc_files ${PROJECTNAME}.xml)

source_group("Source Files" FILES ${source_files})
source_group("DOC Files"    FILES ${doc_files})

include_directories(${ctrlLib_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${source_files} ${doc_files})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ctrlLib)
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${doc_files} DESTINATION ${ICUB_MODULES_INSTALL_DIR})

