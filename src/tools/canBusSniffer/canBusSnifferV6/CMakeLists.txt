# Copyright: (C) 2010 RobotCub Consortium
# Authors: Marco Randazzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME canBusSnifferV6)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

# Find .cpp and .h files automatically.  This is a bit lazy,
# and in principle it would be better to list these files manually.
FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)

# Add our include files into our compiler's search path.
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})

# Create everything needed to build our executable.
ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ctrlLib)
