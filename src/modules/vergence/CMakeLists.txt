IF(WIN32)
  cmake_minimum_required(VERSION 2.6)
ELSE(WIN32)
  cmake_minimum_required(VERSION 2.4)
ENDIF(WIN32)

SET(PROJECTNAME vergence)
PROJECT(${PROJECTNAME})
SET(YARP_DIR "$ENV{YARP_ROOT}" CACHE LOCATION "where is yarp?")

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_DIR}/conf ${YARP_DIR}/conf)
FIND_PACKAGE(IKIN REQUIRED)
FIND_PACKAGE(YARPMATH)
FIND_PACKAGE(LOGPOLAR)

IF (NOT ICUB_FOUND)
    MESSAGE (STATUS "ICUB repository not found; assuming distributed build.")
    MESSAGE (STATUS "If this is not what you want, set ICUB_DIR variable.")
ENDIF (NOT ICUB_FOUND)

SET(CMAKE_MFC_FLAG 2) 

FILE(GLOB folder_source /src/*.cpp)
FILE(GLOB folder_header /include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}
					${YARP_INCLUDE_DIRS}
					${ICUB_INCLUDE_DIRS}
				    ${IKIN_INCLUDE_DIRS} 
					 ${YARPMATH_INCLUDE_DIRS}
                    ${LOGPOLAR_INCLUDE_DIRS} )

LINK_LIBRARIES( ${YARP_LIBRARIES}
				${IKIN_LIBRARIES} 
				${YARPMATH_LIBRARIES}
                ${LOGPOLAR_LIBRARIES})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})
