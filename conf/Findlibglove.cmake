################################################################################
# Copyright (C) 2010 Christian Wressnegger
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

include(findLibrary)

if(NOT LIBGLOVE_FOUND)
  if(WIN32)
    set(Boost_USE_STATIC_LIBS ON)
  endif(WIN32)
  find_package(Boost 1.39.0 REQUIRED COMPONENTS system thread program_options date_time regex)

  if (Boost_FOUND)
    set(libglove_DIR $ENV{libglove_DIR})
    if(libglove_DIR)
      file(TO_CMAKE_PATH $ENV{libglove_DIR} HINTS)
    endif(libglove_DIR)
	
    findLibrary(glove "glove.hpp")
    mergeFindResults(libglove glove)

    set(LIBGLOVE_INCLUDE_DIRS ${LIBGLOVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
    set(LIBGLOVE_LIBRARIES ${LIBGLOVE_LIBRARIES} ${Boost_LIBRARIES})
  endif(Boost_FOUND)

  if(LIBGLOVE_FOUND)
    message(STATUS "Findlibglove found: ${GLOVE_LIBRARIES}")
  else(LIBGLOVE_FOUND)
    message(STATUS "Setting the libglove_DIR environment variable to the install directory might help finding the libraries.")
  endif(LIBGLOVE_FOUND)

endif(NOT LIBGLOVE_FOUND)
