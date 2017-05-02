# Copyright: (C) 2008,2009,2010 RobotCub Consortium
# Authors: Paul Fitzpatrick and Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME iCub_SIM)

IF (MSVC)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /STACK:3145728")
ENDIF ()

PROJECT(${PROJECTNAME})

FILE(GLOB folder_source EyeLidsController.cpp main.cpp)
FILE(GLOB folder_header EyeLidsController.h)

# include code for network wrapper
FILE(GLOB_RECURSE wrapper_source wrapper/*.cpp)
FILE(GLOB_RECURSE wrapper_header wrapper/*.h)
SET(folder_source ${folder_source} ${wrapper_source})
SET(folder_header ${folder_header} ${wrapper_header})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/wrapper)
INCLUDE_DIRECTORIES(${iKin_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${skinDynLib_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})

SET(ICUB_SIM_ENABLE_FAKE TRUE)
SET(ICUB_SIM_ENABLE_ODESDL TRUE)

IF (ICUB_SIM_ENABLE_FAKE)
  # include code for a fake, nonfunctional test version of the simulator
  FILE(GLOB_RECURSE fake_source fake/*.cpp)
  FILE(GLOB_RECURSE fake_header fake/*.h)
  SET(folder_source ${folder_source} ${fake_source})
  SET(folder_header ${folder_header} ${fake_header})
  ADD_DEFINITIONS(-DICUB_SIM_ENABLE_FAKE)
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/fake)
ENDIF ()

IF (ICUB_SIM_ENABLE_ODESDL)
  # include code for standard ODE+SDL simulator
  FILE(GLOB_RECURSE odesdl_source odesdl/*.cpp)
  FILE(GLOB_RECURSE odesdl_header odesdl/*.h)
  SET(folder_source ${folder_source} ${odesdl_source})
  SET(folder_header ${folder_header} ${odesdl_header})

  IF (APPLE)
    ADD_DEFINITIONS(-DHAVE_APPLE_OPENGL_FRAMEWORK)
    MESSAGE(STATUS ${SDL_VERSION})
    ## add SDLMain.m and SDLMain.h required for Mac OS platform
    SET(SDL_MAIN_M "${SDL_INCLUDE_DIR}/SDLMain.m")

    IF (NOT EXISTS "${SDL_MAIN_M}")
      file(GLOB_RECURSE SDL_MAIN_M /usr/local/Cellar/sdl/*.m)
    ENDIF ()

    IF (EXISTS "${SDL_MAIN_M}")
      MESSAGE(STATUS "Using ${SDL_MAIN_M}")
      LIST( APPEND folder_source ${SDL_MAIN_M})
    ELSE ()
      MESSAGE(FATAL_ERROR "On OSX, SDLMain.m is needed.  Yes, it is annoying.")
    ENDIF ()
  ENDIF (APPLE)

  if(ODE_DOUBLE_PRECISION)
    ADD_DEFINITIONS(-DdDOUBLE)
  else()
    ADD_DEFINITIONS(-DdSINGLE)
  endif()
  
  # If glut is set up, we can turn this on
  ADD_DEFINITIONS(-DUSE_GLUT)

  ADD_DEFINITIONS(-DICUB_SIM_ENABLE_ODESDL)

  INCLUDE_DIRECTORIES(${ODE_INCLUDE_DIRS} ${SDL_INCLUDE_DIR})
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/odesdl)

ENDIF ()

find_package(logpolar QUIET)
IF (logpolar_FOUND)
   message(STATUS "iCub Simulator: logpolar library detected and used")
   INCLUDE_DIRECTORIES(${logpolar_INCLUDE_DIRS})
ELSE ()
   ADD_DEFINITIONS(-DOMIT_LOGPOLAR)
ENDIF ()

# import math symbols from standard cmath
add_definitions(-D_USE_MATH_DEFINES)
ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES} iKin)
IF (ICUB_SIM_ENABLE_ODESDL)
  TARGET_LINK_LIBRARIES(${PROJECTNAME} ${SDL_LIBRARY} ${ODE_LIBRARIES})
ENDIF ()

IF (logpolar_FOUND)
  TARGET_LINK_LIBRARIES(${PROJECTNAME} logpolar)
ENDIF ()

TARGET_LINK_LIBRARIES(${PROJECTNAME} skinDynLib)

set(doc_files iCubSimulation.xml)
source_group("DOC Files"    FILES ${doc_files})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin COMPONENT Core)
yarp_install(FILES ${doc_files} DESTINATION ${ICUB_MODULES_INSTALL_DIR})

# added files, touching CMakeLists.txt so GLOB is rerun
# touch 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15






