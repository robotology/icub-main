# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# Based on code by Paul Fitzpatrick

# Cmake code for adding a module to the iCub main build.
# You can assume yarp has been already found and is 
# available.

SET(PROJECTNAME takeOverTheWorld)

PROJECT(${PROJECTNAME})


# Find .cpp and .h files automatically.  This is a bit lazy,
# and in principle it would be better to list these files manually.
FILE(GLOB folder_source src/*.cpp)
FILE(GLOB folder_header include/iCub/*.h)

# Declare groups of source and header files -- makes things pretty in MSVC.
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Add our include files into our compiler's search path.
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${sharksWithLasers_INCLUDE_DIRS})

# Include YARP
include_directories(${YARP_INCLUDE_DIRS})

# Create everything needed to build our executable.
add_executable(${PROJECTNAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECTNAME} sharksWithLasers ${YARP_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)


