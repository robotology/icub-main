# Copyright: 2010 RobotCub Consortium
# Author: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# Wrap kitware's original FindOpenGL. Standardize varibles.
#
# In windows require you set OpenGL_DIR
#
# Set: 
# OpenGL_FOUND
# OpenGL_LIBRARIES
# OpenGL_INCLUDE_DIRS

#message(Find OpenGL form iCub package)

# save current CMAKE_MODULE_PATH, disable it 
# to avoid recursive calls to FindGLUT
set(_CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH "")

set(OpenGL_DIR $ENV{OpenGL_DIR})

if (OpenGL_DIR)
	message(${OpenGL_DIR})
endif(OpenGL_DIR)

find_package(OpenGL)

if (OPENGL_FOUND)
	set(OpenGL_INCLUDE_DIRS ${OPENGL_INCLUDE_DIR})
	set(OpenGL_FOUND TRUE)
	set(OpenGL_LIBRARIES ${OPENGL_LIBRARIES})
endif(OPENGL_FOUND)

# push back original CMAKE_MODULE_PATH
set(CMAKE_MODULE_PATH ${_CMAKE_MODULE_PATH})