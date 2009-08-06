PROJECT(yarpMath)

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

ADD_LIBRARY(yarpMath ${folder_source} ${folder_header})
INSTALL(TARGETS yarpMath DESTINATION lib)
INSTALL(FILES ${folder_header} DESTINATION include/iCub)

