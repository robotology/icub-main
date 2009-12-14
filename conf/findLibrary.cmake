macro(check_modules _prefix _module0)
  # check cached value
  if (NOT DEFINED __pkg_config_checked_${_prefix} OR __pkg_config_checked_${_prefix} LESS ${PKG_CONFIG_VERSION} OR NOT ${_prefix}_FOUND)
    _pkgconfig_parse_options   (_pkg_modules _pkg_is_required "${_module0}" ${ARGN})
    _pkg_check_modules_internal("${_pkg_is_required}" 1 "${_prefix}" ${_pkg_modules})

    _pkgconfig_set(__pkg_config_checked_${_prefix} ${PKG_CONFIG_VERSION})
  endif(NOT DEFINED __pkg_config_checked_${_prefix} OR __pkg_config_checked_${_prefix} LESS ${PKG_CONFIG_VERSION} OR NOT ${_prefix}_FOUND)
endmacro(check_modules)


################################################################################
# Copyright (C) 2009 Christian Wressnegger
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# version 2 as published by the Free Software Foundation.
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software Foundation,
# Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
################################################################################


macro(findLibrary name includeIndicator)

set(name ${name})
set(includeIndicator ${includeIndicator})

if(NOT name)
  message(FATAL_ERROR "The 'findLibrary' function needs the name of the library")
endif(NOT name)

if(NOT includeIndicator)
  message(FATAL_ERROR "The 'findLibrary' function needs an indicator for the to find the library's includes")
endif(NOT includeIndicator)


string(TOUPPER ${name} NAME)
set(libname lib${name})


if(NOT ${NAME}_FOUND)

	if(${NAME}_INCLUDES AND ${NAME}_LIBRARIES)
	   # in cache already
	   set(${NAME}_FIND_QUIETLY TRUE)
	endif(${NAME}_INCLUDES AND ${NAME}_LIBRARIES)
	
	# use pkg-config to get the directories and then use these values
	# in the FIND_PATH() and FIND_LIBRARY() calls
	find_package(PkgConfig)
	if(PKG_CONFIG_FOUND)
	  check_modules(PC_LIB ${libname})
	endif(PKG_CONFIG_FOUND)

	#message(STATUS "-- 1-2 -- ${CMAKE_PREFIX_PATH}")
	#message(STATUS "--  3  -- ${HINTS}")
	#message(STATUS "--  4  -- $ENV{PATH} -- $ENV{LIB}")
	#message(STATUS "--  5  -- ${CMAKE_SYSTEM_PREFIX_PATH}")
	
	find_library(${NAME}_LIBRARIES ${name}
	             HINTS ${HINTS} ${PC_${libname}_LIBDIR} ${PC_${libname}_LIBRARY_DIRS} PATH_SUFFIXES "lib")
	
	get_filename_component(LIBPATH ${${NAME}_LIBRARIES}} PATH)
	if(LIBPATH)
	  string(REGEX REPLACE "(/lib(/(.*))?)$" "/" LIBPATH ${LIBPATH})
	endif(LIBPATH)
	
	find_path(${NAME}_INCLUDES ${includeIndicator}
	          HINTS ${HINTS} ${PC_${libname}_INCLUDEDIR} ${PC_${libname}_INCLUDE_DIRS} ${LIBPATH} PATH_SUFFIXES "include")

	
if(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} LESS 2.6)

  	set(${NAME}_FOUND ${NAME}_LIBRARIES)
	
	set(${NAME}_FOUND ${${NAME}_FOUND})
	set(${NAME}_INCLUDES ${${NAME}_INCLUDES})
	set(${NAME}_INCLUDE_DIRS ${${NAME}_INCLUDES})
	set(${NAME}_LIBRARIES ${${NAME}_LIBRARIES})

else(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} LESS 2.6)

	include(FindPackageHandleStandardArgs)
	find_package_handle_standard_args(${NAME} DEFAULT_MSG ${NAME}_LIBRARIES ${NAME}_INCLUDES)

	set(${NAME}_FOUND ${${NAME}_FOUND} PARENT_SCOPE)
	set(${NAME}_INCLUDES ${${NAME}_INCLUDES} PARENT_SCOPE)
	set(${NAME}_INCLUDE_DIRS ${${NAME}_INCLUDES} PARENT_SCOPE)
	set(${NAME}_LIBRARIES ${${NAME}_LIBRARIES} PARENT_SCOPE)

endif(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} LESS 2.6)

	mark_as_advanced(${NAME}_INCLUDES ${NAME}_LIBRARIES)
	
endif(NOT ${NAME}_FOUND)

endmacro(findLibrary)
