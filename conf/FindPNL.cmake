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
include(mergeFindResults)

if(NOT PNL_FOUND)
  
  set(PNL_DIR $ENV{PNL_DIR})
  if(PNL_DIR)
    file(TO_CMAKE_PATH $ENV{PNL_DIR} HINTS)
  endif(PNL_DIR)

  findLibrary(pnl "pnl_dll.hpp")

  if(PNL_FOUND)
    message(STATUS "FindPNL found: ${PNL_LIBRARIES}")
  else(PNL_FOUND)
    message(STATUS "Setting the PNL_DIR environment variable to the install directory might help finding the libraries.")
  endif(PNL_FOUND)

endif(NOT PNL_FOUND)
