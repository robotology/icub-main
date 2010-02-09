#/*
# * Copyright (C) 2010 Stephen Hart. All rights reserved.
# * 
# */
cmake_minimum_required(VERSION 2.6)

SET(PROJECTNAME controlBasisModules)
PROJECT(${PROJECTNAME})
SET(CB_RESOURCE_MODULE controlBasisResource)

FIND_PACKAGE(ICUB QUIET)
IF (NOT ICUB_FOUND)
   MESSAGE (STATUS "ICUB repository not found; assuming distributed build.")
   MESSAGE (STATUS "If this is not what you want, set ICUB_DIR variable.")
ENDIF (NOT ICUB_FOUND)

FIND_PACKAGE(IKIN REQUIRED)
SET(CONTROLBASIS_DIR "${ICUB_DIR}/src/controlBasis")
FIND_PACKAGE(CONTROLBASIS REQUIRED)

LINK_LIBRARIES(${CONTROLBASIS_LIBRARIES} ${IKIN_LIBRARIES})
INCLUDE_DIRECTORIES(${CONTROLBASIS_INCLUDE_DIRS} ${IKIN_INCLUDE_DIRS})

ADD_EXECUTABLE(${CB_RESOURCE_MODULE} controlBasisResource.cpp)

INSTALL_TARGETS(/bin controlBasisResource )

