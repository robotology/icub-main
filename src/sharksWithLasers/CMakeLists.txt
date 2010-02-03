
# This is just an example library, that does not do anything much.  It
# is intended to be a quick way to understand current good practice
# for making iCub libraries.  This hasn't really stabilized yet, so if
# you see something that is out of date, please feel free to fix it

SET(PROJECTNAME sharksWithLasers)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(ICUB QUIET)
IF (NOT ICUB_FOUND)
   MESSAGE (STATUS "ICUB repository not found; assuming distributed build.")
   MESSAGE (STATUS "If this is not what you want, set ICUB_DIR variable.")
ENDIF (NOT ICUB_FOUND)

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

# Add a postfix for debug library only in Windows
IF (WIN32 AND NOT CYGWIN)
   SET_TARGET_PROPERTIES(${PROJECTNAME} PROPERTIES DEBUG_POSTFIX "d")
ENDIF (WIN32 AND NOT CYGWIN)

INSTALL(TARGETS ${PROJECTNAME} DESTINATION lib)
INSTALL(FILES ${folder_header} DESTINATION include/iCub)

