# Copyright: (C) 2010 RobotCub Consortium
# Authors: Alessandro Scalzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME frameGrabberGui2)
PROJECT(${PROJECTNAME})

FILE(GLOB_RECURSE folder_header include/*.h)
FILE(GLOB_RECURSE folder_source src/*.cpp)
SOURCE_GROUP("Header Files" FILES ${folder_header})
SOURCE_GROUP("Source Files" FILES ${folder_source})

ADD_DEFINITIONS(${GTK2_DEFINITIONS})
INCLUDE_DIRECTORIES(${GTK2_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)
ADD_EXECUTABLE(${PROJECTNAME} ${folder_header} ${folder_source})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${GTK2_LIBRARIES} ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)
