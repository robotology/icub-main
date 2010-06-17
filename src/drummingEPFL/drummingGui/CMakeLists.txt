IF(WIN32)
cmake_minimum_required(VERSION 2.6)
ELSE(WIN32)
cmake_minimum_required(VERSION 2.4)
ENDIF(WIN32)

SET(PROJECTNAME drummingGui)
PROJECT(${PROJECTNAME})

FIND_PACKAGE(GtkMM REQUIRED)

ADD_DEFINITIONS(${GTKMM_C_FLAGS})
LINK_LIBRARIES(${GTKMM_LINK_FLAGS})
INCLUDE_DIRECTORIES(${GTKMM_INCLUDE_DIR})

 
FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h )
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

add_definitions( "/wd4250 /wd4996 /nologo" )

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})
TARGET_LINK_LIBRARIES(${PROJECTNAME} ${GTKMM_LINK_FLAGS})

