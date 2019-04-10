# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

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

include(AddInstallRPATHSupport)
add_install_rpath_support(LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"       # Libraries
                          BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"       # Binaries
                                   "${CMAKE_INSTALL_FULL_LIBDIR}/iCub"  # Plugins
                          INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}"
                          USE_LINK_PATH)

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
