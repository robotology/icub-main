#/*
# * Copyright (C) 2009 Stephen Hart. All rights reserved.
# * 
# */
cmake_minimum_required(VERSION 2.6)

SET(PROJECTNAME controlBasis)
PROJECT(${PROJECTNAME})

SET(NESTED_BUILD FALSE)

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)
FIND_PACKAGE(IKIN REQUIRED)
FIND_PACKAGE(REINFORCEMENTLEARNING)
IF (NOT REINFORCEMENTLEARNING_FOUND)
   MESSAGE (STATUS "REINFORCEMENTLEARNING library not found; assuming distributed build.")
ENDIF (NOT REINFORCEMENTLEARNING_FOUND)


# Find .cpp and .h files automatically.  This is a bit lazy,
# and in principle it would be better to list these files manually.
FILE(GLOB folder_source 
  resources/*.cpp
  potentialFunctions/*.cpp
  jacobians/*.cpp
  control/*.cpp
  )
FILE(GLOB folder_header 
  include/*.h
  resources/*.h
  potentialFunctions/*.h
  jacobians/*.h
  control/*.h
  )

# Declare groups of source and header files -- makes things pretty in MSVC.
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(
  ${IKIN_INCLUDE_DIRS}
  ${REINFORCEMENTLEARNING_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/resources
  ${PROJECT_SOURCE_DIR}/jacobians
  ${PROJECT_SOURCE_DIR}/potentialFunctions
  ${PROJECT_SOURCE_DIR}/control
)

LINK_DIRECTORIES(${LIB_DIR})
EXPORT_LIBRARY_DEPENDENCIES(${PROJECT_BINARY_DIR}/${PROJECTNAME}LibraryDepends.cmake)
ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})
#TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${IKIN_LIBRARIES} ${REINFORCEMENTLEARNING_LIBRARIES})
INSTALL(TARGETS ${PROJECTNAME} DESTINATION ${ICUB_DIR}/lib)

SUBDIRS(modules
  util)


