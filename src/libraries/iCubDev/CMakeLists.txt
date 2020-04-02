# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(iCubDev)

# Find .cpp and .h files automatically.  This is a bit lazy,
# and in principle it would be better to list these files manually.
FILE(GLOB folder_source src/*.cpp)
FILE(GLOB folder_header include/iCub/*.h)

# Declare groups of source and header files -- makes things pretty in MSVC.
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Add our include files into our compiler's search path.
include_directories(${PROJECT_SOURCE_DIR}/include)

# Create everything needed to build our executable.
add_library(${PROJECT_NAME} ${folder_source} ${folder_header})
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
target_link_libraries(${PROJECT_NAME} YARP::YARP_os
                                      YARP::YARP_dev)

icub_export_library(${PROJECT_NAME}
                    INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include 
                    DESTINATION include/iCub 
                    FILES ${folder_header})




