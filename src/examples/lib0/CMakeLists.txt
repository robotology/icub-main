# Copyright: (C) 2009 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# Based on code by Paul Fitzpatrick

# Cmake code for adding a library to the iCub main build.
# You can assume yarp has been already found and is 
# available.

SET(PROJECTNAME lib0)

PROJECT(${PROJECTNAME})

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

icub_export_library(${PROJECTNAME} ${PROJECT_SOURCE_DIR}/include ${folder_header})


