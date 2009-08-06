PROJECT(rfwr)

FIND_PACKAGE(ICUB QUIET)
IF (NOT ICUB_FOUND)
   MESSAGE (STATUS "ICUB repository not found; assuming distributed build.")
   MESSAGE (STATUS "If this is not what you want, set ICUB_DIR variable.")
ENDIF (NOT ICUB_FOUND)
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(YARPMATH REQUIRED)

INCLUDE_DIRECTORIES(${YARPMATH_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR})
LINK_LIBRARIES(${YARPMATH_LIBRARIES})

FILE(GLOB folder_header *.h)
FILE(GLOB folder_source *.cpp)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

ADD_LIBRARY(rfwr ${folder_source} ${folder_header})

ADD_EXECUTABLE(test_rfwr "example/main.cpp")
TARGET_LINK_LIBRARIES(test_rfwr rfwr)
TARGET_LINK_LIBRARIES(test_rfwr ${YARPMATH_LIBRARIES})

INSTALL(TARGETS rfwr DESTINATION lib)
INSTALL(FILES ${folder_header} DESTINATION include/iCub)

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})
