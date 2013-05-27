# Try to locate the IPOPT library
#
# If the IPOPT_DIR is set, try to locate the package in the given
# directory, otherwise use pkg-config to locate it
#
# Create the following variables:
# IPOPT_INCLUDE_DIRS - Directories to include to use IPOPT
# IPOPT_LIBRARIES    - Default library to link against to use IPOPT
# IPOPT_DEFINITIONS  - Flags to be added to linker's options
# IPOPT_FOUND        - If false, don't try to use IPOPT
#
# Deprecated variables:
# IPOPT_LINK_FLAGS   - = IPOPT_DEFINITIONS, for compatibility

# Copyright (C) 2008-2010  RobotCub Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


if(APPLE)

    set(IPOPT_DIR $ENV{IPOPT_DIR} CACHE PATH "Path to IPOPT build directory")

    find_package(PkgConfig)
    if(PKG_CONFIG_FOUND)
        pkg_check_modules(IPOPT ipopt)
        link_directories(${IPOPT_LIBRARY_DIRS}) # vital on Macs, but not many
                                             # ipopt-using programs do this

        foreach(arg ${IPOPT_LDFLAGS})
            set(IPOPT_LINK_FLAGS "${IPOPT_LINK_FLAGS} ${arg}")
        endforeach(arg ${IPOPT_LDFLAGS})
    endif()

elseif(UNIX)

    # in linux if the env var IPOPT_DIR is not set
    # we know we are dealing with an installed iCub package
    set(IPOPT_DIR_TEST $ENV{IPOPT_DIR})
    if(IPOPT_DIR_TEST)
        set(IPOPT_DIR $ENV{IPOPT_DIR} CACHE PATH "Path to IPOPT build directory")
    else()
        set(IPOPT_DIR /usr            CACHE PATH "Path to IPOPT build directory")
    endif()

    set(IPOPT_INCLUDE_DIRS ${IPOPT_DIR}/include/coin)
    find_library(IPOPT_LIBRARIES ipopt ${IPOPT_DIR}/lib
                                       ${IPOPT_DIR}/lib/coin
                                       NO_DEFAULT_PATH)

    if(IPOPT_LIBRARIES)
        find_file(IPOPT_DEP_FILE ipopt_addlibs_cpp.txt ${IPOPT_DIR}/share/doc/coin/Ipopt
                                                       ${IPOPT_DIR}/share/coin/doc/Ipopt
                                                       NO_DEFAULT_PATH)
        mark_as_advanced(IPOPT_DEP_FILE)

        if(IPOPT_DEP_FILE)
            # parse the file and acquire the dependencies
            file(READ ${IPOPT_DEP_FILE} IPOPT_DEP)
            string(REGEX REPLACE "-[^l][^ ]* " "" IPOPT_DEP ${IPOPT_DEP})
            string(REPLACE "-l"                "" IPOPT_DEP ${IPOPT_DEP})
            string(REPLACE "\n"                "" IPOPT_DEP ${IPOPT_DEP})
            string(REPLACE "ipopt"             "" IPOPT_DEP ${IPOPT_DEP})       # remove any possible auto-dependency
            separate_arguments(IPOPT_DEP)

            # use the find_library command in order to prepare rpath correctly
            foreach(LIB ${IPOPT_DEP})
                find_library(SEARCH_FOR_IPOPT_${LIB} ${LIB} ${IPOPT_DIR}/lib
                                                            ${IPOPT_DIR}/lib/coin
                                                            ${IPOPT_DIR}/lib/coin/ThirdParty
                                                            NO_DEFAULT_PATH)
                if(SEARCH_FOR_IPOPT_${LIB})
                    # handle non-system libraries (e.g. coinblas)
                    set(IPOPT_LIBRARIES ${IPOPT_LIBRARIES} ${SEARCH_FOR_IPOPT_${LIB}})
                else(SEARCH_FOR_IPOPT_${LIB})
                    # handle system libraries (e.g. gfortran)
                    set(IPOPT_LIBRARIES ${IPOPT_LIBRARIES} ${LIB})
                endif(SEARCH_FOR_IPOPT_${LIB})
                mark_as_advanced(SEARCH_FOR_IPOPT_${LIB})
            endforeach(LIB)
        endif()
    endif()

    set(IPOPT_LINK_FLAGS "")

# Windows platforms
else()

    set(IPOPT_DIR $ENV{IPOPT_DIR} CACHE PATH "Path to IPOPT build directory")

    set(IPOPT_INCLUDE_DIRS ${IPOPT_DIR}/include/coin)
    find_library(IPOPT_LIBRARIES_RELEASE libipopt  ${IPOPT_DIR}/lib
                                                   ${IPOPT_DIR}/lib/coin
                                                   NO_DEFAULT_PATH)
    find_library(IPOPT_LIBRARIES_DEBUG   libipoptD ${IPOPT_DIR}/lib
                                                   ${IPOPT_DIR}/lib/coin
                                                   NO_DEFAULT_PATH)

    if(IPOPT_LIBRARIES_RELEASE AND IPOPT_LIBRARIES_DEBUG)
        set(IPOPT_LIBRARIES optimized ${IPOPT_LIBRARIES_RELEASE} debug ${IPOPT_LIBRARIES_DEBUG})
    elseif(IPOPT_LIBRARIES_RELEASE)
        set(IPOPT_LIBRARIES ${IPOPT_LIBRARIES_RELEASE})
    elseif(IPOPT_LIBRARIES_DEBUG)
        set(IPOPT_LIBRARIES ${IPOPT_LIBRARIES_DEBUG})
    endif()

    set(IPOPT_LIBRARIES_RELEASE "")
    set(IPOPT_LIBRARIES_DEBUG "")

    if(MSVC)
        set(IPOPT_LINK_FLAGS "/NODEFAULTLIB:libcmt.lib;libcmtd.lib")
    else()
        set(IPOPT_LINK_FLAGS "")
    endif()

endif()

if(IPOPT_LIBRARIES)
   set(IPOPT_FOUND TRUE)
else()
   set(IPOPT_FOUND FALSE)
   set(IPOPT_INCLUDE_DIRS "")
   set(IPOPT_LIBRARIES "")
   set(IPOPT_LINK_FLAGS "")
endif()
