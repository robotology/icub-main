PROJECT(yarpMath)

SET(ICUB_DIR "$ENV{ICUB_ROOT}" CACHE LOCATION "where is icub root?")
#SET(INSTALL_DIR "${ICUB_DIR}")
SET(INSTALL_DIR "${CMAKE_SOURCE_DIR}")

FILE(GLOB folder_header *.h)
FILE(GLOB folder_source *.cpp)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

ADD_LIBRARY(yarpMath ${folder_source} ${folder_header})
INSTALL(TARGETS yarpMath DESTINATION "${INSTALL_DIR}/lib")

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
