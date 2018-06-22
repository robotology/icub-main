# Copyright (C) 2010 RobotCub Consortium
# Authors: Andrea del Prete, Alexander Schmitz
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME skinManager)
PROJECT(${PROJECTNAME})

# Search for source code.
FILE(GLOB folder_source src/*.cpp)
FILE(GLOB folder_header include/iCub/skinManager/*.h)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/src/libraries/icubmod/skinLib)

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} skinDynLib)
INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

