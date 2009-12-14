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


function(mergeFindResults target) # [package1 [package2 [package3 ... ]]])

if(NOT target)
  message(FATAL_ERROR "The 'mergeFindResults' function at least needs the target name")
endif(NOT target)

string(TOUPPER ${target} TARGET)


set(FOUND TRUE)
set(LIBRARIES "")

foreach(arg ${ARGN})
  string(TOUPPER ${arg} ARG)
  if(${ARG}_FOUND)
    set(INCLUDES ${INCLUDES} ${${ARG}_INCLUDES})
    set(LIBRARIES ${LIBRARIES} ${${ARG}_LIBRARIES})
    set(${ARG}_FOUND)
    set(${ARG}_INCLUDES)
    set(${ARG}_LIBRARIES)
  else(${ARG}_FOUND)
    set(FOUND FALSE)
  endif(${ARG}_FOUND)
endforeach(arg ${ARGN})

if(INCLUDES)
  list(REMOVE_DUPLICATES INCLUDES)
endif(INCLUDES)

set(${TARGET}_INCLUDES ${INCLUDES})
set(${TARGET}_LIBRARIES ${LIBRARIES})

set(FOUND ${${TARGET}_LIBRARIES})
find_package_handle_standard_args(${TARGET} DEFAULT_MSG ${TARGET}_LIBRARIES)


set(${TARGET}_FOUND ${${TARGET}_FOUND} PARENT_SCOPE)
set(${TARGET}_INCLUDES ${INCLUDES} PARENT_SCOPE)
set(${TARGET}_INCLUDE_DIRS ${INCLUDES} PARENT_SCOPE)
set(${TARGET}_LIBRARIES ${LIBRARIES} PARENT_SCOPE)

endfunction(mergeFindResults)
