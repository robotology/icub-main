# Usually, all you'll need to do is set the name of your library here.
# The rest of the file can remain unchanged in most cases.
SET(LIB_TARGET "BML")   # name used in ADD_LIBRARY(...)
SET(LIB_PKG "BML")      # name you want in FIND_PACKAGE(...)

# We expect a <LIBRARY>_DIR variable to be available, pointing to
# the directory with this library in it.
SET(LIB_DIR ${${LIB_PKG}_DIR})

MESSAGE(STATUS "BML Library")


IF (NESTED_BUILD)
  SET(BML_LIBRARIES BML)
ELSE (NESTED_BUILD)
  FIND_LIBRARY(${LIB_PKG}_LIBRARIES ${LIB_TARGET} ${BML_DIR}/lib)
  MESSAGE(STATUS "Looking at ${BML_DIR}/lib")
  

  IF (NOT ${LIB_PKG}_LIBRARIES)

    # We may be on a system with "Release" and "Debug" sub-configurations
    FIND_LIBRARY(${LIB_PKG}_LIBRARIES_RELEASE ${LIB_TARGET} ${LIB_DIR}/release NO_DEFAULT_PATH)
    FIND_LIBRARY(${LIB_PKG}_LIBRARIES_DEBUG ${LIB_TARGET} ${LIB_DIR}/debug NO_DEFAULT_PATH)

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

SET(BML_INCLUDE_DIRS ${BML_DIR}/include)
