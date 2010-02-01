SET(KEYWORD "artoolkittracker")

# Start a project.
PROJECT(${KEYWORD})  

# Find YARP.  Point the YARP_DIR environment variable at your build.

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP)
FIND_PACKAGE(OpenCV)
FIND_PACKAGE(ARToolkit)

# Search for source code.
FILE(GLOB folder_source ./src/*.cpp ./src/*.cc ./src/*.c)
FILE(GLOB folder_header ./include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})  

# Automatically add include directories if needed. 
FOREACH(header_file ${folder_header})   
	GET_FILENAME_COMPONENT(p ${header_file} PATH)   
	INCLUDE_DIRECTORIES(${p}) 
ENDFOREACH(header_file ${folder_header})  


INCLUDE_DIRECTORIES(${ARTOOLKIT_INCLUDE_DIR} ${OPENCV_INCLUDE_DIR})
LINK_LIBRARIES(${ARTOOLKIT_AR_LIBRARY} ${OPENCV_LIBRARIES})

# Set up our main executable. 
IF (folder_source)   
	ADD_EXECUTABLE(${KEYWORD} ${folder_source} ${folder_header}) 
ELSE (folder_source)   
	MESSAGE(FATAL_ERROR "No source code files found. Please add something") 
ENDIF (folder_source) 

INSTALL_TARGETS(../../bin ${KEYWORD})
