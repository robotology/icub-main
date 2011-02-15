# Copyright: (C) 2010 RobotCub Consortium
# Authors: Marco Randazzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME joystickCtrl)

PROJECT(${PROJECTNAME})

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)

IF (APPLE)
  ADD_DEFINITIONS(-DHAVE_APPLE_OPENGL_FRAMEWORK)
 ## add SDLMain.m and SDLMain.h required for Mac OS platform
 LIST( APPEND folder_source ${SDL_INCLUDE_DIR}/SDLMain.m )
ENDIF (APPLE)

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}
		    ${SDL_INCLUDE_DIR}
                    ${YARP_INCLUDE_DIRS})

SET(folder_source main.cpp)

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${SDL_LIBRARY} ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)