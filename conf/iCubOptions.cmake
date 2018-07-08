# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

MACRO(icub_install_with_rpath)
    #########################################################################
    # Control setting an rpath
    if (NOT MSVC)
        option(ICUB_INSTALL_WITH_RPATH "Set an rpath after installing the executables" FALSE)
        #mark_as_advanced(ICUB_ENABLE_FORCE_RPATH)
    endif (NOT MSVC)

    # By default do not build with rpath.
    # If this flag is true then all the variables related to RPATH are ignored
    if(ICUB_INSTALL_WITH_RPATH)
        # Maintain back-compatibility
        if(${CMAKE_MINIMUM_REQUIRED_VERSION} VERSION_GREATER "2.8.12")
            message(AUTHOR_WARNING "CMAKE_MINIMUM_REQUIRED_VERSION is now ${CMAKE_MINIMUM_REQUIRED_VERSION}. This check can be removed.")
        endif()
        if(CMAKE_VERSION VERSION_LESS 2.8.12)
            set(CMAKE_INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}")
            set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_FULL_LIBDIR}")
        else()
            # This is relative RPATH for libraries built in the same project
            # I assume that the directory is
            #  - install_dir/something for binaries
            #  - install_dir/lib for libraries
            # in this way if libraries and executables are moved together everything will continue to work
            set(ICUB_FULL_BIN_DIR "${CMAKE_INSTALL_PREFIX}/bin")
            set(ICUB_FULL_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib")

            file(RELATIVE_PATH _rel_path "${ICUB_FULL_BIN_DIR}" "${ICUB_FULL_LIB_DIR}")
            if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
                set(CMAKE_INSTALL_RPATH "@loader_path/${_rel_path}")
            else()
                set(CMAKE_INSTALL_RPATH "\$ORIGIN/${_rel_path}")
            endif()
        endif()

        # Enable RPATH on OSX. This also suppress warnings on CMake >= 3.0
        set(CMAKE_MACOSX_RPATH 1)

        # When building, don't use the install RPATH already
        # (but later on when installing)
        set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

        # Add the automatically determined parts of the RPATH
        # which point to directories outside the build tree to the install RPATH
        set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

    endif(ICUB_INSTALL_WITH_RPATH)

ENDMACRO(icub_install_with_rpath)

##### options
if(MSVC)
    MESSAGE(STATUS "Running on windows")

    # ACE uses a bunch of functions MSVC warns about.
    # The warnings make sense in general, but not in this case.
    # this gets rids of deprecated unsafe crt functions
    add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
    # this gets rid of warning about deprecated POSIX names
    add_definitions(-D_CRT_NONSTDC_NO_DEPRECATE)
    # Traditionally, we add "d" postfix to debug libraries

    # Trying to disable: warning C4355: 'this' : used ...
    # with no luck.
    ##add_definitions("/wd4355")
    ##set(CMAKE_CXX_FLAGS "/wd4355 ${CMAKE_CXX_FLAGS}")

    set(CMAKE_DEBUG_POSTFIX "d")
endif(MSVC)

if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
        "Choose the type of build, recommanded options are: Debug or Release" FORCE)
    endif()
    set(ICUB_BUILD_TYPES "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${ICUB_BUILD_TYPES})
endif()

########################################################################
# settings for rpath

icub_install_with_rpath() #from icubHelpers

#########################################################################
# Shared library option (hide on windows for now)

if(NOT MSVC OR NOT CMAKE_VERSION VERSION_LESS 3.4)
    option(ICUB_SHARED_LIBRARY "Compile shared libraries rather than static libraries" FALSE)
    if(ICUB_SHARED_LIBRARY)
        set(BUILD_SHARED_LIBS ON)
        if(MSVC)
            set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
        endif()
    endif()
endif()

#########################################################################
# Compile libraries using -fPIC to produce position independent code
# since CMake 2.8.10 the variable CMAKE_POSITION_INDEPENDENT_CODE is
# used by cmake to determine whether position indipendent code
# executable and library targets should be created
# For older versions the position independent code is handled in
# iCubHelpers.cmake, in the icub_export_library macro (and obviously
# only for targets exported using that macro)

if(CMAKE_VERSION VERSION_GREATER "2.8.9")
    set(CMAKE_POSITION_INDEPENDENT_CODE "TRUE")
endif()


#########################################################################
# C++11 flags

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" CXX_HAS_STD_CXX11)
check_cxx_compiler_flag("-std=c++0x" CXX_HAS_STD_CXX0X)
if(CXX_HAS_STD_CXX11)
    set(CXX11_FLAGS "-std=c++11")
elseif(CXX_HAS_STD_CXX0X)
    set(CXX11_FLAGS "-std=c++0x")
else()
    set(CXX11_FLAGS)
endif()
