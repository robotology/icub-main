#reduce warning level with cmake 2.6
#to be improved, Lorenzo Natale
IF(WIN32)
cmake_minimum_required(VERSION 2.6)
#disable annoying warnings about use of deprecated c functions
ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /NODEFAULTLIB:libcmt.lib;libcmtd.lib")

ELSE(WIN32)
cmake_minimum_required(VERSION 2.4)
ENDIF(WIN32)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

PROJECT(iCub)

SET(NESTED_BUILD TRUE)

SET(ICUB_DIR ${PROJECT_SOURCE_DIR} CACHE LOCATION "Where is ICUB root?")
INCLUDE(${PROJECT_SOURCE_DIR}/ICUBOptions.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/ICUBConfig.cmake)

SET(BUILD_IN_ROOT TRUE CACHE BOOL "Do you want to build libs and bins in $ICUB_ROOT/lib and $ICUB_ROOT/bin?")
SET(USE_ICUB_MOD FALSE CACHE BOOL "Use separated icub_mod lib")
SET(COMPILE_FOR_TESTING FALSE CACHE BOOL "Set this if compiling for testing purposes")
MARK_AS_ADVANCED(COMPILE_FOR_TESTING)  # not for general use

SET(COMPILE_NEW_FRAMEGRABBERGUI TRUE CACHE BOOL "Compile new framegrabbergui")
SET(EXPERIMENTAL_APP_INSTALL FALSE CACHE BOOL "Install scripts and config files (experimental)")

IF(BUILD_IN_ROOT)
SET(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
# this doesn't happen automatically for makefiles
MAKE_DIRECTORY(${LIBRARY_OUTPUT_PATH})
MAKE_DIRECTORY(${EXECUTABLE_OUTPUT_PATH})
ENDIF(BUILD_IN_ROOT)

SUBDIRS(src)
IF (EXPERIMENTAL_APP_INSTALL)
  SUBDIRS(app)
ENDIF(EXPERIMENTAL_APP_INSTALL)

# make dox target - not very smart
ADD_CUSTOM_TARGET(dox COMMAND doxygen ${PROJECT_SOURCE_DIR}/conf/Doxyfile.txt)

# master configuration file
INSTALL(FILES ICUB_ROOT.ini DESTINATION ${CMAKE_INSTALL_PREFIX})
