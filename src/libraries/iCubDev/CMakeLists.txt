SET(PROJECTNAME iCubDev)
PROJECT(${PROJECTNAME})

# Find .cpp and .h files automatically.  This is a bit lazy,
# and in principle it would be better to list these files manually.
FILE(GLOB folder_source src/*.cpp)
FILE(GLOB folder_header include/iCub/*.h)

# Declare groups of source and header files -- makes things pretty in MSVC.
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Add our include files into our compiler's search path.
include_directories(${PROJECT_SOURCE_DIR}/include ${YARP_INCLUDE_DIRS})

# Create everything needed to build our executable.
add_library(${PROJECTNAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})

icub_export_library(${PROJECTNAME} 
                    INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include 
                    DESTINATION include/iCub 
                    VERBOSE
                    FILES ${folder_header})




