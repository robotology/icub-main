PROJECT(demo_logpolar)

SET(YARP_DIR "$ENV{YARP_ROOT}" CACHE LOCATION "where is yarp?")
SET(CMAKE_MODULE_PATH "$ENV{YARP_ROOT}/conf")

INCLUDE(YarpProgram)

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP)
FIND_PACKAGE(GtkPlus)
IF (NOT GtkPlus_FOUND)
    MESSAGE(ERROR "gtk+ not found, cannot compile the application!")
ENDIF (NOT GtkPlus_FOUND)

ADD_DEFINITIONS(${GTKPLUS_C_FLAGS})
LINK_LIBRARIES(${GTKPLUS_LINK_FLAGS})

INCLUDE_DIRECTORIES(${GTKPLUS_INCLUDE_DIR})
INCLUDE_DIRECTORIES("$ENV{ICUB_ROOT}/include")

LINK_LIBRARIES(debug "$ENV{ICUB_ROOT}/lib/lib_iCub_logpolard" optimized "$ENV{ICUB_ROOT}/lib/lib_iCub_logpolar")

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h include/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

IF (WIN32 AND NOT CYGWIN)
    ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)
    ADD_EXECUTABLE(demo_logpolar ${folder_source} ${folder_header})
    YarpProgram(demo_logpolar NO_CONSOLE) #damn!
ELSE (WIN32 AND NOT CYGWIN)
    ADD_EXECUTABLE(demo_logpolar ${folder_source} ${folder_header})
    YarpProgram(demo_logpolar CONSOLE)
ENDIF (WIN32 AND NOT CYGWIN)

INSTALL_TARGETS(/bin demo_logpolar)
