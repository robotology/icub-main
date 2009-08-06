IF (COMPILE_DEVICE_LIBRARY)
  PREPARE_DEVICE(ecan TYPE EsdCan
    INCLUDE EsdCan.h 
    WRAPPER controlboard)
ENDIF (COMPILE_DEVICE_LIBRARY)

IF (NOT SKIP_ecan)
  FIND_PACKAGE(YARP REQUIRED)
  
  IF(WIN32 AND NOT CYGWIN)
    FIND_PATH(inc_esdapi ntcan.h PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../esdCanAPi/winnt/include)
	
    FIND_LIBRARY(lib_esdapi ntcan ${CMAKE_CURRENT_SOURCE_DIR}/../esdCanAPi/winnt/lib)
  ELSE(WIN32 AND NOT CYGWIN)
    FIND_PATH(inc_esdapi ntcan.h PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../esdCanAPi/linux/include)
    FIND_LIBRARY(lib_esdapi ntcan ${CMAKE_CURRENT_SOURCE_DIR}/../esdCanAPi/linux/lib)
  ENDIF(WIN32 AND NOT CYGWIN)

  IF(inc_esdapi AND lib_esdapi)
    INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
    INCLUDE_DIRECTORIES(${inc_esdapi})
    LINK_LIBRARIES(${lib_esdapi})

    ADD_LIBRARY(esdCan EsdCan.cpp EsdCan.h)
  ELSE (inc_esdapi AND lib_esdapi)
    MESSAGE(SEND_ERROR "Cannot find ntcan lib, check esdCanApi installation instructions")
  ENDIF (inc_esdapi AND lib_esdapi)
ENDIF (NOT SKIP_ecan)
