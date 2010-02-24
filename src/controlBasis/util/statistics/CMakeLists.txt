#/*
# * Copyright (C) 2010 Stephen Hart. All rights reserved.
# * 
# */
cmake_minimum_required(VERSION 2.6)

PROJECT(statistics_test)

FIND_PACKAGE(YARP QUIET)

FIND_PACKAGE(ICUB QUIET)
IF (NOT ICUB_FOUND)
   MESSAGE (STATUS "ICUB repository not found; assuming distributed build.")
   MESSAGE (STATUS "If this is not what you want, set ICUB_DIR variable.")
ENDIF (NOT ICUB_FOUND)

INCLUDE_DIRECTORIES( . )

ADD_EXECUTABLE(statisticsTest main.cpp NonParametricProbabilityDistribution.cpp)


