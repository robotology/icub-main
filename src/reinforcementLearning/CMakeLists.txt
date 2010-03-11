#/*
# * Copyright (C) 2010 Stephen Hart. All rights reserved.
# * 
# */
cmake_minimum_required(VERSION 2.6)

SET(PROJECTNAME reinforcementLearning)
PROJECT(${PROJECTNAME})
SET(RL_MODULE reinforcementLearning)

ADD_DEFINITIONS(-O3 -Wno-deprecated -DTIXML_USE_STL)

FIND_PACKAGE(YARP QUIET)
FIND_PACKAGE(ICUB)

INCLUDE_DIRECTORIES(tinyxml)

# Find .cpp and .h files automatically.  This is a bit lazy,
# and in principle it would be better to list these files manually.
FILE(GLOB folder_source 
  *.cpp
  tinyxml/*.cpp
  )
FILE(GLOB folder_header 
  *.h
  tinyxml/*.h
  )

# Declare groups of source and header files -- makes things pretty in MSVC.
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${IKIN_LIBRARIES} ${CTRLLIB_LIBRARIES} ${IPOPTLIB_LIBRARIES} ${REINFORCEMENTLEARNING_LIBRARIES})

INSTALL_TARGETS(/bin ${RL_MODULE})
INSTALL(TARGETS ${PROJECTNAME} DESTINATION ${ICUB_DIR}/lib)

