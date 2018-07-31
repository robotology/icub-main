# Copyright: (C) 2016 icubFacility IIT - Istituto Italiano di Tecnologia
# Authors: Marco Randazzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(imageCropper)
set(PROJECTNAME imageCropper)

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

add_executable(${PROJECTNAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)