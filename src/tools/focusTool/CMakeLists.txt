# Copyright: (C) 2011 RobotCub Consortium
# Authors: Alessandro Scalzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME focusTool)
PROJECT(${PROJECTNAME})

FILE(GLOB_RECURSE folder_header include/*.h)
FILE(GLOB_RECURSE folder_source src/*.cpp)
SOURCE_GROUP("Header Files" FILES ${folder_header})
SOURCE_GROUP("Source Files" FILES ${folder_source})

ADD_DEFINITIONS(${GtkMM_C_FLAGS})
ADD_DEFINITIONS(${Gthread_C_FLAGS})

INCLUDE_DIRECTORIES(${GtkMM_INCLUDE_DIRS} ${Gthread_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)
ADD_EXECUTABLE(${PROJECTNAME} ${folder_header} ${folder_source})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${GtkMM_LIBRARIES} ${Gthread_LIBRARIES} ${GtkPlus_LIBRARIES} ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)
