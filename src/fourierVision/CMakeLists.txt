
# This is just an example library, that does not do anything much.  It
# is intended to be a quick way to understand current good practice
# for making iCub libraries.  This hasn't really stabilized yet, so if
# you see something that is out of date, please feel free to fix it

SET(PROJECTNAME fourierVision)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(ICUB QUIET)
IF (NOT ICUB_FOUND)
   MESSAGE (STATUS "ICUB repository not found; assuming distributed build.")
   MESSAGE (STATUS "If this is not what you want, set ICUB_DIR variable.")
ENDIF (NOT ICUB_FOUND)

FIND_PACKAGE(YARP REQUIRED)

FIND_PACKAGE(LOGPOLAR REQUIRED)
LINK_LIBRARIES(${LOGPOLAR_LIBRARIES})
INCLUDE_DIRECTORIES(${LOGPOLAR_INCLUDE_DIRS})

# Find .cpp and .h files automatically.  This is a bit lazy,
# and in principle it would be better to list these files manually.
FILE(GLOB folder_source src/*.cpp)
FILE(GLOB folder_header include/iCub/*.h)

# Declare groups of source and header files -- makes things pretty in MSVC.
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Add our include files into our compiler's search path.
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

# Create everything needed to build our executable.
ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})

