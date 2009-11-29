if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

PROJECT (iCubGui)

IF(WIN32)
	CMAKE_MINIMUM_REQUIRED (VERSION 2.6.0)
ELSE(WIN32)
	CMAKE_MINIMUM_REQUIRED (VERSION 2.4.0)
ENDIF(WIN32)

# Configurable install variables, use CMAKE_INSTALL_PREFIX to install into target directory
# removed conflicts with install prefix set by the main cmake file
# SET(CMAKE_INSTALL_PREFIX "." CACHE PATH "Root install path" FORCE)

#SET(YARP_DIR "$ENV{YARP_ROOT}" CACHE LOCATION "where is yarp?")
FIND_PACKAGE(YARP)

# Initialize and search for QT4 libraries
SET(QT_USE_QTOPENGL TRUE)
FIND_PACKAGE(Qt3)
IF(QT_FOUND)
	# Search for OpenGL and GLUT
	FIND_PACKAGE(OpenGL)
	IF(OPENGL_FOUND)
		FIND_PACKAGE(GLUT)
		IF(GLUT_FOUND)
			INCLUDE_DIRECTORIES(src ${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR} ${QT_INCLUDE_DIR})
			SUBDIRS(src)
		ELSE(GLUT_FOUND)
			MESSAGE("GLUT not found, won't compile iCubGui")
		ENDIF(GLUT_FOUND)
	ELSE(OPENGL_FOUND)
		MESSAGE("OpenGL not found, won't compile iCubGui")
	ENDIF(OPENGL_FOUND)
ELSE(QT_FOUND)
	MESSAGE("Qt3 not found, won't compile iCubGui")
ENDIF(QT_FOUND)
