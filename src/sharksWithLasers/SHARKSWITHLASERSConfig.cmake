
# Usually, all you'll need to do is set the name of your library here.
# The rest of the file can remain unchanged in most cases.
SET(LIB_TARGET "sharksWithLasers")   # name used in ADD_LIBRARY(...)
SET(LIB_PKG "SHARKSWITHLASERS")  # name you want in FIND_PACKAGE(...)

# Write a message, just so user knows what FIND_PACKAGE is calling
MESSAGE(STATUS "Using ${LIB_PKG}Config.cmake")

# We expect a <LIBRARY>_DIR variable to be available, pointing to
# the directory with this library in it.
SET(LIB_DIR ${${LIB_PKG}_DIR})

# Tell the user what include directories need to be added to use this library.
SET(${LIB_PKG}_INCLUDE_DIRS ${LIB_DIR}/include)


# Are we compiling as part of the ICub project, or separately?
IF (NESTED_BUILD)

  # This a global build, so we do not need to supply the full path
  # and filename(s) of the library.  We can just use the CMake target name.
  # CMake itself knows what exactly the library is called on this system.
  IF (WIN32 AND NOT CYGWIN)
     FIND_LIBRARY(${LIB_PKG}_LIBRARIES_RELEASE ${LIB_TARGET}  ${ICUB_DIR}/lib/Release NO_DEFAULT_PATH)
     FIND_LIBRARY(${LIB_PKG}_LIBRARIES_DEBUG   ${LIB_TARGET}d ${ICUB_DIR}/lib/Debug   NO_DEFAULT_PATH)
     SET (${LIB_PKG}_LIBRARIES optimized ${${LIB_PKG}_LIBRARIES_RELEASE} debug ${${LIB_PKG}_LIBRARIES_DEBUG})
  ELSE (WIN32 AND NOT CYGWIN)
     SET (${LIB_PKG}_LIBRARIES ${LIB_TARGET})
  ENDIF (WIN32 AND NOT CYGWIN)

ELSE (NESTED_BUILD)

  # this a distributed build, so we have to pin down the library path
  # and filename exactly.

  FIND_LIBRARY(${LIB_PKG}_LIBRARIES ${LIB_TARGET} ${LIB_DIR})

  IF (NOT ${LIB_PKG}_LIBRARIES)

    # We may be on a system with "Release" and "Debug" sub-configurations
    FIND_LIBRARY(${LIB_PKG}_LIBRARIES_RELEASE ${LIB_TARGET} 
		 ${LIB_DIR}/Release NO_DEFAULT_PATH)
    FIND_LIBRARY(${LIB_PKG}_LIBRARIES_DEBUG ${LIB_TARGET}d 
		 ${LIB_DIR}/Debug NO_DEFAULT_PATH)

    IF (${LIB_PKG}_LIBRARIES_RELEASE AND NOT ${LIB_PKG}_LIBRARIES_DEBUG)
	SET(${LIB_PKG}_LIBRARIES ${${LIB_PKG}_LIBRARIES_RELEASE} CACHE PATH "release version of library" FORCE)
    ENDIF (${LIB_PKG}_LIBRARIES_RELEASE AND NOT ${LIB_PKG}_LIBRARIES_DEBUG)

    IF (${LIB_PKG}_LIBRARIES_DEBUG AND NOT ${LIB_PKG}_LIBRARIES_RELEASE)
	SET(${LIB_PKG}_LIBRARIES ${${LIB_PKG}_LIBRARIES_DEBUG} CACHE PATH "debug version of library" FORCE)
    ENDIF (${LIB_PKG}_LIBRARIES_DEBUG AND NOT ${LIB_PKG}_LIBRARIES_RELEASE)

    IF (${LIB_PKG}_LIBRARIES_DEBUG AND ${LIB_PKG}_LIBRARIES_RELEASE)
	SET(${LIB_PKG}_LIBRARIES 
			optimized ${${LIB_PKG}_LIBRARIES_RELEASE}
			debug ${${LIB_PKG}_LIBRARIES_DEBUG}  CACHE PATH "debug and release version of library" FORCE)
    ENDIF (${LIB_PKG}_LIBRARIES_DEBUG AND ${LIB_PKG}_LIBRARIES_RELEASE)

  ENDIF (NOT ${LIB_PKG}_LIBRARIES)

ENDIF (NESTED_BUILD)

