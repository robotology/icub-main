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

if(NOT FLOCKOFBIRDS_FOUND)
  
  set(FlockOfBirds_DIR $ENV{FlockOfBirds_DIR})
  if(FlockOfBirds_DIR)
    file(TO_CMAKE_PATH $ENV{FlockOfBirds_DIR} HINTS)
  endif(FlockOfBirds_DIR)

  findLibrary(Bird "Bird.h")
  mergeFindResults(FlockOfBirds Bird)

  if(FLOCKOFBIRDS_FOUND)
    message(STATUS "FindFlockOfBirds found: ${FLOCKOFBIRDS_LIBRARIES}")
  else(FLOCKOFBIRDS_FOUND)
    message(STATUS "Setting the FlockOfBirds_DIR environment variable to the install directory might help finding the libraries.")
  endif(FLOCKOFBIRDS_FOUND)

endif(NOT FLOCKOFBIRDS_FOUND)
