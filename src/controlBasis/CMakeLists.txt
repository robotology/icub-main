#/*
# * Copyright (C) 2009 Stephen Hart. All rights reserved.
# * 
# */

PROJECT(ControlBasis)

FIND_PACKAGE(YARP)
FIND_PACKAGE(OpenCV)
FIND_PACKAGE(ICUB)
FIND_PACKAGE(GtkMM)

FIND_PACKAGE(IKIN REQUIRED)
INCLUDE_DIRECTORIES(${IKIN_INCLUDE_DIRS}
  ${GtkMM_INCLUDE_DIRS}
  include)

#When we type "make install"
SET(BIN_DIR /bin)
SET(LIB_DIR /lib)
SET(INC_DIR /include)
SET(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})

LINK_DIRECTORIES(${LIB_DIR}
  ${IKIN_LIBRARIES}
  ${GtkMM_LIBRARIES}
  )

#TARGET_LINK_LIBRARIES(${IKIN_LIBRARIES})

SUBDIRS(
	resources
	potentialFunctions
	jacobians
	control
	modules
	app
	tests
	)

