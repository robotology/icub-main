# Copyright: (C) 2008,2009,2010 RobotCub Consortium
# Authors: Paul Fitzpatrick and Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME iCub_SIM)

PROJECT(${PROJECTNAME})

FILE(GLOB folder_source EyeLidsController.cpp main.cpp)
FILE(GLOB folder_header EyeLidsController.h)

# include code for network wrapper
FILE(GLOB_RECURSE wrapper_source wrapper/*.cpp)
FILE(GLOB_RECURSE wrapper_header wrapper/*.h)
SET(folder_source ${folder_source} ${wrapper_source})
SET(folder_header ${folder_header} ${wrapper_header})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/wrapper)

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

  IF (NOT DEFINED USE_ODE_DOUBLE)
    IF (NOT CMAKE_CROSS_COMPILING)
      INCLUDE(CheckCXXSourceRuns)
      SET(CMAKE_REQUIRED_INCLUDES ${ODE_INCLUDE_DIRS})
      SET(CMAKE_REQUIRED_LIBRARIES ${ODE_LIBRARIES})
      CHECK_CXX_SOURCE_RUNS("extern \"C\" int dCheckConfiguration(const char*str); \n int main() { return dCheckConfiguration(\"ODE_double_precision\")?0:1; }\n" ODE_IS_DOUBLE)
      # should return 1 if ODE_double_precision
    ELSE (CMAKE_CROSS_COMPILING)
      SET(ODE_IS_DOUBLE FALSE)
    ENDIF ()
    SET(USE_ODE_DOUBLE ${ODE_IS_DOUBLE} CACHE BOOL "Select false if compiled ODE in single precision, or true for double precision")
  ENDIF ()

  IF (USE_ODE_DOUBLE)
    ADD_DEFINITIONS(-DdDOUBLE)
  ELSE ()
    ADD_DEFINITIONS(-DdSINGLE)
  ENDIF (USE_ODE_DOUBLE)
  
  # If glut is set up, we can turn this on
  ADD_DEFINITIONS(-DUSE_GLUT)

  ADD_DEFINITIONS(-DICUB_SIM_ENABLE_ODESDL)

  INCLUDE_DIRECTORIES(${ODE_INCLUDE_DIRS} ${SDL_INCLUDE_DIR})
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/odesdl)

ENDIF ()


OPTION(ICUB_SIM_OMIT_LOGPOLAR "Omit logpolar support from simulator" FALSE)
IF (ICUB_SIM_OMIT_LOGPOLAR)
  ADD_DEFINITIONS(-DOMIT_LOGPOLAR)
ELSE ()
  INCLUDE_DIRECTORIES(${logpolar_INCLUDE_DIRS})
ENDIF ()

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES})
IF (ICUB_SIM_ENABLE_ODESDL)
  TARGET_LINK_LIBRARIES(${PROJECTNAME} ${SDL_LIBRARY} ${ODE_LIBRARIES})
ENDIF ()

IF (NOT ICUB_SIM_OMIT_LOGPOLAR)
  TARGET_LINK_LIBRARIES(${PROJECTNAME} logpolar)
ENDIF ()

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin COMPONENT Core)

# added files, touching CMakeLists.txt so GLOB is rerun
# touch 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15






