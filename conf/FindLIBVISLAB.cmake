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

if(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} LESS 2.6)

  message(STATUS "-- FindLIBVISLAB requires CMake >=2.6")

else(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} LESS 2.6)

  include(findLibrary)
  include(mergeFindResults)

  if(NOT LIBVISLAB_FOUND)

    if($ENV{LIBVISLAB_DIR})
      file(TO_CMAKE_PATH $ENV{LIBVISLAB_DIR} HINTS)
    endif($ENV{LIBVISLAB_DIR})

    findLibrary(vislab "vislab/util/all.h")
    findLibrary(vislab_YARP "vislab/yarp/util/all.h")

    mergeFindResults(LIBVISLAB vislab_YARP vislab)

    if(LIBVISLAB_FOUND)
      message(STATUS "FindLIBVISLAB found: ${LIBVISLAB_LIBRARIES}")
    endif(LIBVISLAB_FOUND)

  endif(NOT LIBVISLAB_FOUND)

endif(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} LESS 2.6)
