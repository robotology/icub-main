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


macro(mergeFindResults target) # [package1 [package2 [package3 ... ]]])

set(target ${target})

if(NOT target)
  message(FATAL_ERROR "The 'mergeFindResults' function at least needs the target name")
endif(NOT target)

string(TOUPPER ${target} TARGET)


set(FOUND 1)
set(LIBRARIES "")

foreach(arg ${ARGN})
  string(TOUPPER ${arg} ARG)
  if(${ARG}_FOUND)
    if(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION} LESS 2.4.8)
      set(b -1)
    elseif(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION} LESS 2.4.8)
      list(FIND INCLUDES "${${ARG}_INCLUDES}" b)
    endif(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION} LESS 2.4.8)
    if(b LESS 0)
      set(INCLUDES ${INCLUDES} ${${ARG}_INCLUDES})
   	endif(b LESS 0)
    set(LIBRARIES ${LIBRARIES} ${${ARG}_LIBRARIES})
    set(${ARG}_FOUND)
    set(${ARG}_INCLUDES)
    set(${ARG}_LIBRARIES)
  else(${ARG}_FOUND)
    set(FOUND 0)
  endif(${ARG}_FOUND)
endforeach(arg ${ARGN})

set(${TARGET}_INCLUDES ${INCLUDES})
set(${TARGET}_LIBRARIES ${LIBRARIES})

if(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} LESS 2.6)

  set(${TARGET}_FOUND ${TARGET}_LIBRARIES)

  set(${TARGET}_FOUND ${${TARGET}_FOUND})
  set(${TARGET}_INCLUDES ${INCLUDES})
  set(${TARGET}_INCLUDE_DIRS ${INCLUDES})
  set(${TARGET}_LIBRARIES ${LIBRARIES})

else(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} LESS 2.6)
  
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(${TARGET} DEFAULT_MSG ${TARGET}_LIBRARIES)

  set(${TARGET}_FOUND ${${TARGET}_FOUND} PARENT_SCOPE)
  set(${TARGET}_INCLUDES ${INCLUDES} PARENT_SCOPE)
  set(${TARGET}_INCLUDE_DIRS ${INCLUDES} PARENT_SCOPE)
  set(${TARGET}_LIBRARIES ${LIBRARIES} PARENT_SCOPE)

endif(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} LESS 2.6)

endmacro(mergeFindResults)
