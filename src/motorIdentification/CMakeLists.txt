SET(PROJECTNAME motorIdentification)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

# Find .cpp and .h files automatically.  This is a bit lazy,
# and in principle it would be better to list these files manually.
SET(folder_source main.cpp) 
SET(folder_header )
# FILE(GLOB folder_header *.h)

# Declare groups of source and header files -- makes things pretty in MSVC.

SOURCE_GROUP("source files" FILES ${folder_source})
SOURCE_GROUP("header files" FILES ${folder_header})

# Create everything needed to build our executable.
ADD_EXECUTABLE(${PROJECTNAME} ${folder_header} ${folder_source})
INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)  
