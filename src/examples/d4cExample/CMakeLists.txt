# Copyright: (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ilaria Gori, Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME d4cExample)

project(${PROJECTNAME})

set(folder_source main.cpp)

source_group("Source Files" FILES ${folder_source})

include_directories(${d4c_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${folder_source})
target_link_libraries(${PROJECTNAME} d4c ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)

