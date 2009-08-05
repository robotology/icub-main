SET(KEYWORD "rdout")

PROJECT(${KEYWORD})

FIND_PACKAGE(YARP REQUIRED)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h include/iCub/*.hh)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
)

LINK_LIBRARIES(
    
)

# Set up our main executable.
IF (folder_source)
  ADD_LIBRARY(${KEYWORD} ${folder_source} ${folder_header})
ELSE (folder_source)
  MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)

INSTALL_TARGETS(/lib ${KEYWORD})


 
