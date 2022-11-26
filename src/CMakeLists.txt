# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

## Order here is important, libraries go first because they set variables
## that are used by the other modules. 

option(ICUBMAIN_COMPILE_LIBRARIES "Enable icub-main libraries." ON)
mark_as_advanced(ICUBMAIN_COMPILE_LIBRARIES)

option(ICUBMAIN_COMPILE_CORE "Enable icub-main core." ON)
mark_as_advanced(ICUBMAIN_COMPILE_CORE)

option(ICUBMAIN_COMPILE_TOOLS "Enable icub-main tools." ON)
mark_as_advanced(ICUBMAIN_COMPILE_TOOLS)

option(ICUBMAIN_COMPILE_MODULES "Enable icub-main modules." ON)
mark_as_advanced(ICUBMAIN_COMPILE_MODULES)

option(BUILD_TESTING "Enable unittest." OFF)

if (ICUBMAIN_COMPILE_LIBRARIES)
add_subdirectory(libraries)
endif()

if (ICUBMAIN_COMPILE_CORE)
add_subdirectory(core)
endif()

if (ICUBMAIN_COMPILE_TOOLS)
add_subdirectory(tools)
endif()

if (ICUBMAIN_COMPILE_MODULES)
add_subdirectory(modules)
endif()

if (BUILD_TESTING)
  if(NOT BUILD_SHARED_LIBS)
    add_subdirectory(unittest)
  endif()
endif()

