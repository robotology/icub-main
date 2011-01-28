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
	
	# disable: warning C4355: 'this' : used ...
    add_definitions(/wd4355)
    set(CMAKE_DEBUG_POSTFIX "d")
endif(MSVC)

if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE "Release" CACHE STRING 
        "Choose the type of build, recommanded options are: Debug or Release")
endif(NOT CMAKE_BUILD_TYPE)
# Hide variable to MSVC users, since it is not needed
 if (MSVC)
        mark_as_advanced(CMAKE_BUILD_TYPE)
endif(MSVC)

########################################################################
# settings for rpath


#########################################################################
# Control setting an rpath
if (NOT MSVC)
	set(ICUB_INSTALL_WITH_RPATH FALSE CACHE BOOL "Set an rpath after installing the executables")
	#mark_as_advanced(ICUB_ENABLE_FORCE_RPATH)
endif (NOT MSVC)


if (ICUB_INSTALL_WITH_RPATH )
  # when building, don't use the install RPATH already
  # (but later on when installing), this tells cmake to relink 
  # at install, so in-tree binaries have correct rpath
  SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

  SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
  SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)  
endif (ICUB_INSTALL_WITH_RPATH )
#########################################################################

