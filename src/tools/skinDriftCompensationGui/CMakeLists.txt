# Copyright (C) 2010 RobotCub Consortium
# Authors: Andrea del Prete
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)

set(PROJECTNAME skinDriftCompensationGui)
PROJECT(${PROJECTNAME})

#optionally: use cmake find scripts provided by YARP and iCub
set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})
set(CMAKE_INSTALL_PREFIX "$ENV{ICUB_DIR}" CACHE PATH "Install path prefix, pre-appended onto install directory." FORCE)

#set up gthread
FIND_PACKAGE(Gthread REQUIRED)
ADD_DEFINITIONS(${GTHREAD_C_FLAGS})
LINK_LIBRARIES(${Gthread_LIBRARIES})
INCLUDE_DIRECTORIES(${Gthread_INCLUDE_DIRS})

# set up to use GTK+
FIND_PACKAGE(GtkPlus REQUIRED)
ADD_DEFINITIONS(${GTKPLUS_C_FLAGS})
LINK_LIBRARIES(${GtkPlus_LIBRARIES})
LINK_DIRECTORIES(${GTKPLUS_LIBRARY_DIR})
INCLUDE_DIRECTORIES(${GtkPlus_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})

# Search for source code.
FILE(GLOB folder_source src/*.cpp)
FILE(GLOB folder_header include/iCub/skinDriftCompensationGui/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})
ADD_EXECUTABLE(${PROJECTNAME}  ${folder_source} ${folder_header})

target_link_libraries(${PROJECTNAME} 
	${YARP_LIBRARIES} 
	${GTKPLUS_LINK_FLAGS} 
	${GtkPlus_LIBRARIES})

if(WIN32)
	INSTALL(TARGETS ${PROJECTNAME} RUNTIME DESTINATION bin/release)
	SET_TARGET_PROPERTIES(${PROJECTNAME}  PROPERTIES COMPILE_FLAGS /wd4996)
else(WIN32)
	INSTALL(TARGETS ${PROJECTNAME} RUNTIME DESTINATION bin)
endif(WIN32)


