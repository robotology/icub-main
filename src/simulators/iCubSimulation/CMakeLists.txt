SET(PROJECTNAME iCub_SIM)

PROJECT(${PROJECTNAME})

SET(USE_ODE_DOUBLE TRUE CACHE BOOL "Select false if compiled ODE in single precision")

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)

IF (APPLE)
  ADD_DEFINITIONS(-DHAVE_APPLE_OPENGL_FRAMEWORK)
 ## add SDLMain.m and SDLMain.h required for Mac OS platform
 LIST( APPEND folder_source ${SDL_INCLUDE_DIR}/SDLMain.m )
ENDIF (APPLE)

IF (USE_ODE_DOUBLE)
  ADD_DEFINITIONS(-DdDOUBLE)
ENDIF (USE_ODE_DOUBLE)

# If glut is set up, we can turn this on
ADD_DEFINITIONS(-DUSE_GLUT)

# Using the new configuration method right from the start
ADD_DEFINITIONS(-DEXPERIMENTAL_CONFIG_METHOD)

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}
                    ${logPolar_INCLUDE_DIRS}
				    ${ODE_INCLUDE_DIRS}
					${SDL_INCLUDE_DIR}
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME}
                      ${SDL_LIBRARY}
			          ${ODE_LIBRARIES}
                      ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

# added files, touching CMakeLists.txt so GLOB is rerun


