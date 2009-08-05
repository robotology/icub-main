SET(PROJECTNAME logpolarRemapper)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB QUIET)
IF (NOT ICUB_FOUND)
  MESSAGE (STATUS "ICUB repository not found; assuming distributed build.")
  MESSAGE (STATUS "If this is not what you want, set ICUB_DIR variable.")
ENDIF (NOT ICUB_FOUND)
# support library for iCub devices and interfaces
FIND_PACKAGE(ICUBDEV REQUIRED)
LINK_LIBRARIES(${ICUBDEV_LIBRARIES})
INCLUDE_DIRECTORIES(${ICUBDEV_INCLUDE_DIRS})

IF(USE_ICUB_MOD)
  ADD_DEFINITIONS(-DUSE_ICUB_MOD)
  FIND_PACKAGE(icubmod)
  IMPORT_DEVICES(${ICUB_DIR}/src/logpolarRemapper/include/iCub/drivers.h icubmod)
ENDIF (USE_ICUB_MOD)

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
ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})
INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

# Makes VS a bit quiter wrt warnings.
IF(MSVC)
  SET_TARGET_PROPERTIES(${PROJECTNAME} PROPERTIES COMPILE_FLAGS /wd4996)
ENDIF(MSVC)