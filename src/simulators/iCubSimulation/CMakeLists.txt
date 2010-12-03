SET(PROJECTNAME iCub_SIM)

PROJECT(${PROJECTNAME})

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)

IF (APPLE)
  ADD_DEFINITIONS(-DHAVE_APPLE_OPENGL_FRAMEWORK)
 ## add SDLMain.m and SDLMain.h required for Mac OS platform
 LIST( APPEND folder_source ${SDL_INCLUDE_DIR}/SDLMain.m )
ENDIF (APPLE)

IF (NOT DEFINED USE_ODE_DOUBLE)
  IF (NOT CMAKE_CROSS_COMPILING)
    INCLUDE(CheckCXXSourceRuns)
    SET(CMAKE_REQUIRED_INCLUDES ${ODE_INCLUDE_DIRS})
    SET(CMAKE_REQUIRED_LIBRARIES ${ODE_LIBRARIES})
    CHECK_CXX_SOURCE_RUNS("#include <ode/ode.h>\n int main() { return dCheckConfiguration(\"ODE_double_precision\")?0:1; }\n" ODE_IS_DOUBLE)
    # should return 1 if ODE_double_precision
  ELSE (CMAKE_CROSS_COMPILING)
    SET(ODE_IS_DOUBLE FALSE)
  ENDIF ()
  SET(USE_ODE_DOUBLE ${ODE_IS_DOUBLE} CACHE BOOL "Select false if compiled ODE in single precision, or true for double precision")
ENDIF ()

IF (USE_ODE_DOUBLE)
  ADD_DEFINITIONS(-DdDOUBLE)
ENDIF (USE_ODE_DOUBLE)

# If glut is set up, we can turn this on
ADD_DEFINITIONS(-DUSE_GLUT)

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}
                    ${logpolar_INCLUDE_DIRS}
				    ${ODE_INCLUDE_DIRS}
					${SDL_INCLUDE_DIR}
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} logpolar
                                     ${SDL_LIBRARY}
			                         ${ODE_LIBRARIES}
                                     ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

# added files, touching CMakeLists.txt so GLOB is rerun
# touch 1 2 3 4 5 6 7 8



