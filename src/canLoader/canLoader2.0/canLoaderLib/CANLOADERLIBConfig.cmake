SET(LIB_TARGET "canLoaderLib")   # name used in ADD_LIBRARY(...)
SET(LIB_PKG "CANLOADERLIB")  # name you want in FIND_PACKAGE(...)

# Write a message, just so user knows what FIND_PACKAGE is calling
MESSAGE(STATUS "Using ${LIB_PKG}Config.cmake")
SET(LIB_DIR ${${LIB_PKG}_DIR})
SET(${LIB_PKG}_INCLUDE_DIRS ${LIB_DIR})

# Are we compiling as part of the ICub project, or separately?
IF (NESTED_BUILD)
  # This a global build, so we do not need to supply the full path
  # and filename(s) of the library.  We can just use the CMake target name.
  # CMake itself knows what exactly the library is called on this system.
  SET(${LIB_PKG}_LIBRARIES ${LIB_TARGET})
ELSE (NESTED_BUILD)
  # this a distributed build, so we have to pin down the library path
  # and filename exactly.
  FIND_LIBRARY(${LIB_PKG}_LIBRARIES ${LIB_TARGET} ${LIB_DIR})
  IF (NOT ${LIB_PKG}_LIBRARIES)
    # We may be on a system with "Release" and "Debug" sub-configurations
    FIND_LIBRARY(${LIB_PKG}_LIBRARIES_RELEASE ${LIB_TARGET} 
		 ${LIB_DIR}/Release NO_DEFAULT_PATH)
    FIND_LIBRARY(${LIB_PKG}_LIBRARIES_DEBUG ${LIB_TARGET} 
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