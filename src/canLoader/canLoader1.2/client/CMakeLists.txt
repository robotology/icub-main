PROJECT(canLoaderClient)

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP)

FIND_PACKAGE(GtkPlus)
IF (NOT GtkPlus_FOUND)
    MESSAGE(ERROR "gtk+ not found, won't compile canLoaderClient")
ENDIF (NOT GtkPlus_FOUND)

ADD_DEFINITIONS(${GTKPLUS_C_FLAGS})
LINK_LIBRARIES(${GTKPLUS_LINK_FLAGS})

INCLUDE_DIRECTORIES(${GTKPLUS_INCLUDE_DIR})

SET(folder_source main.cpp)
SOURCE_GROUP("Source Files" FILES ${folder_source})

IF (WIN32 AND NOT CYGWIN)
   #win32 here creates an executable without console
    ADD_EXECUTABLE(canLoaderClient WIN32 ${folder_source} ${folder_header})
ELSE (WIN32 AND NOT CYGWIN)
    ADD_EXECUTABLE(canLoaderClient ${folder_source} ${folder_header})
ENDIF (WIN32 AND NOT CYGWIN)

INSTALL_TARGETS(/bin canLoaderClient) 
