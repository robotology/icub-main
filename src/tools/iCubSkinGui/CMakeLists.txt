# Copyright: (C) 2010 RobotCub Consortium
# Authors: Alessandro Scalzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME iCubSkinGui)
PROJECT(${PROJECTNAME})

FILE(GLOB_RECURSE folder_header src/include/*.h)
FILE(GLOB_RECURSE folder_source src/*.cpp)
SOURCE_GROUP("Header Files" FILES ${folder_header})
SOURCE_GROUP("Source Files" FILES ${folder_source})

ADD_DEFINITIONS(${GTKPLUS_C_FLAGS})

#set up gthread
FIND_PACKAGE(Gthread REQUIRED)
ADD_DEFINITIONS(${GTHREAD_C_FLAGS})
LINK_LIBRARIES(${Gthread_LIBRARIES})
INCLUDE_DIRECTORIES(${Gthread_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src
                    ${ctrlLib_INCLUDE_DIRS}
                    ${GtkPlus_INCLUDE_DIRS}
	            ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_header} ${folder_source})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ctrlLib
				     icubmod
				     ${GtkPlus_LIBRARIES}
				     ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)


