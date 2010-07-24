#message(Find GLUT form iCub package)

# save current CMAKE_MODULE_PATH, disable it 
# to avoid recursive calls to FindGLUT
set(_CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH "")

set(GLUT_DIR $ENV{GLUT_DIR})

#message(${GLUT_DIR})
	
if (WIN32)
    FIND_PATH(GLUT_INCLUDE_DIR NAMES glut.h 
    PATHS ${GLUT_DIR})
  FIND_LIBRARY(GLUT_LIBRARIES NAMES glut glut32
    PATHS
    ${GLUT_DIR}
	)
	#message(${GLUT_LIBRARIES})
	if (GLUT_INCLUDE_DIR AND GLUT_LIBRARIES)
		set(GLUT_FOUND TRUE CACHE BOOL "GLUT found?")
	endif(GLUT_INCLUDE_DIR AND GLUT_LIBRARIES)
endif(WIN32)
	
if(NOT GLUT_FOUND)
	#message(calling kitware find)
	find_package(GLUT)
endif(NOT GLUT_FOUND)

if (GLUT_FOUND)
	set(GLUT_INCLUDE_DIRS ${GLUT_INCLUDE_DIR})
endif(GLUT_FOUND)

# push back original CMAKE_MODULE_PATH
set(CMAKE_MODULE_PATH ${_CMAKE_MODULE_PATH})


