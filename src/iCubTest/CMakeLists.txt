PROJECT(iCubTest)

IF(WIN32)
	CMAKE_MINIMUM_REQUIRED (VERSION 2.6.0)
ELSE(WIN32)
	CMAKE_MINIMUM_REQUIRED (VERSION 2.4.0)
ENDIF(WIN32)

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB QUIET)

IF (NOT ICUB_FOUND)
   MESSAGE (STATUS "ICUB repository not found; assuming distributed build.")
   MESSAGE (STATUS "If this is not what you want, set ICUB_DIR variable.")
ENDIF (NOT ICUB_FOUND)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})

FILE(GLOB folder_header *.h)
FILE(GLOB folder_source *.cpp)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

ADD_LIBRARY(iCubTestLib ${folder_source} ${folder_header})
ADD_EXECUTABLE(iCubTest ${folder_source} ${folder_header})

INSTALL(TARGETS iCubTestLib DESTINATION lib)
INSTALL_TARGETS(/bin iCubTest)
INSTALL(FILES ${folder_header} DESTINATION include/iCub)
