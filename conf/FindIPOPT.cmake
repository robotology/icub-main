# Copyright: 2008-2010 RobotCub Consortium
# Author: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Created:
# IPOPT_INCLUDE_DIRS - Directories to include to use IPOPT
# IPOPT_LIBRARIES    - Default library to link against to use IPOPT
# IPOPT_LINK_FLAGS   - Flags to be added to linker's options
# IPOPT_FOUND        - If false, don't try to use IPOPT

IF(APPLE)
 
   SET(IPOPT_DIR $ENV{IPOPT_DIR} CACHE PATH "Path to IPOPT build directory")
 
   FIND_PACKAGE(PkgConfig)
   IF(PKG_CONFIG_FOUND)
     PKG_CHECK_MODULES(IPOPT ipopt)
     LINK_DIRECTORIES(${IPOPT_LIBRARY_DIRS}) # vital on Macs, but not many
                                             # ipopt-using programs do this

     foreach(arg ${IPOPT_LDFLAGS})
       set(IPOPT_LINK_FLAGS "${IPOPT_LINK_FLAGS} ${arg}")
     endforeach(arg ${IPOPT_LDFLAGS})

   ENDIF()
 
ELSEIF(UNIX)

   # in linux if the env var IPOPT_DIR is not set
   # we know we are dealing with an installed iCub package
   SET(IPOPT_DIR_TEST $ENV{IPOPT_DIR})
   IF(IPOPT_DIR_TEST)
      SET(IPOPT_DIR $ENV{IPOPT_DIR} CACHE PATH "Path to IPOPT build directory")
   ELSE()
      SET(IPOPT_DIR /usr            CACHE PATH "Path to IPOPT build directory")
   ENDIF()

   SET(IPOPT_INCLUDE_DIRS ${IPOPT_DIR}/include/coin)
   FIND_LIBRARY(IPOPT_LIBRARIES ipopt ${IPOPT_DIR}/lib
                                      ${IPOPT_DIR}/lib/coin
                                      NO_DEFAULT_PATH)
   
   IF(IPOPT_LIBRARIES)
      FIND_FILE(IPOPT_DEP_FILE ipopt_addlibs_cpp.txt ${IPOPT_DIR}/share/doc/coin/Ipopt
                                                     ${IPOPT_DIR}/share/coin/doc/Ipopt
                                                     NO_DEFAULT_PATH)
      MARK_AS_ADVANCED(IPOPT_DEP_FILE)

      IF(IPOPT_DEP_FILE)
         # parse the file and acquire the dependencies
         FILE(READ ${IPOPT_DEP_FILE} IPOPT_DEP)
         STRING(REGEX REPLACE "-[^l][^ ]* " "" IPOPT_DEP ${IPOPT_DEP})
         STRING(REPLACE "-l"                "" IPOPT_DEP ${IPOPT_DEP})
         STRING(REPLACE "\n"                "" IPOPT_DEP ${IPOPT_DEP})
         STRING(REPLACE "ipopt"             "" IPOPT_DEP ${IPOPT_DEP})       # remove any possible auto-dependency
         SEPARATE_ARGUMENTS(IPOPT_DEP)
   
         # use the find_library command in order to prepare rpath correctly 
         FOREACH(LIB ${IPOPT_DEP})
            FIND_LIBRARY(SEARCH_FOR_IPOPT_${LIB} ${LIB} ${IPOPT_DIR}/lib
                                                        ${IPOPT_DIR}/lib/coin
                                                        ${IPOPT_DIR}/lib/coin/ThirdParty
                                                        NO_DEFAULT_PATH)
            IF(SEARCH_FOR_IPOPT_${LIB})
               # handle non-system libraries (e.g. coinblas)
               SET(IPOPT_LIBRARIES ${IPOPT_LIBRARIES} ${SEARCH_FOR_IPOPT_${LIB}})
            ELSE(SEARCH_FOR_IPOPT_${LIB})
               # handle system libraries (e.g. gfortran)
               SET(IPOPT_LIBRARIES ${IPOPT_LIBRARIES} ${LIB})
            ENDIF(SEARCH_FOR_IPOPT_${LIB})
            MARK_AS_ADVANCED(SEARCH_FOR_IPOPT_${LIB})
         ENDFOREACH(LIB)
      ENDIF()
   ENDIF()
   
   SET(IPOPT_LINK_FLAGS "")

# Windows platforms
ELSE()

   SET(IPOPT_DIR $ENV{IPOPT_DIR} CACHE PATH "Path to IPOPT build directory")

   SET(IPOPT_INCLUDE_DIRS ${IPOPT_DIR}/include/coin)
   FIND_LIBRARY(IPOPT_LIBRARIES_RELEASE libipopt  ${IPOPT_DIR}/lib 
                                                  ${IPOPT_DIR}/lib/coin
                                                  NO_DEFAULT_PATH)
   FIND_LIBRARY(IPOPT_LIBRARIES_DEBUG   libipoptD ${IPOPT_DIR}/lib
                                                  ${IPOPT_DIR}/lib/coin
                                                  NO_DEFAULT_PATH)

   IF(IPOPT_LIBRARIES_RELEASE AND IPOPT_LIBRARIES_DEBUG)
      SET(IPOPT_LIBRARIES optimized ${IPOPT_LIBRARIES_RELEASE} debug ${IPOPT_LIBRARIES_DEBUG})
   ELSE()
      IF(IPOPT_LIBRARIES_RELEASE)
         SET(IPOPT_LIBRARIES ${IPOPT_LIBRARIES_RELEASE})
      ELSE()
         IF(IPOPT_LIBRARIES_DEBUG)
            SET(IPOPT_LIBRARIES ${IPOPT_LIBRARIES_DEBUG})
         ENDIF()
      ENDIF()
   ENDIF()

   SET(IPOPT_LIBRARIES_RELEASE "")
   SET(IPOPT_LIBRARIES_DEBUG "")

   IF(MSVC)
       SET(IPOPT_LINK_FLAGS "/NODEFAULTLIB:libcmt.lib;libcmtd.lib")
   ELSE()
       SET(IPOPT_LINK_FLAGS "")
   ENDIF()

ENDIF()

IF(IPOPT_LIBRARIES)
   SET(IPOPT_FOUND TRUE)
ELSE()
   SET(IPOPT_FOUND FALSE)
   SET(IPOPT_INCLUDE_DIRS "")
   SET(IPOPT_LIBRARIES "")
   SET(IPOPT_LINK_FLAGS "")
ENDIF()
