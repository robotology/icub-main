IF (COMPILE_DEVICE_LIBRARY)
  PREPARE_DEVICE(pcan TYPE PlxCan
    INCLUDE PlxCan.h 
    WRAPPER "")
ENDIF (COMPILE_DEVICE_LIBRARY)

IF (NOT SKIP_pcan)
  FIND_PACKAGE(YARP REQUIRED)
  FIND_PATH(inc_plxapi PexApi.h PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../plxCanApi/linux/include)

  FIND_LIBRARY(lib_plxapi PlxApi ${CMAKE_CURRENT_SOURCE_DIR}/../plxCanApi/linux/lib)

  IF(inc_plxapi AND lib_plxapi)
    INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
    INCLUDE_DIRECTORIES(${inc_plxapi})
    LINK_LIBRARIES(${lib_plxapi})

	#plx-api related define
	ADD_DEFINITIONS(-DPLX_LITTLE_ENDIAN -DPLX_LINUX -DGCC)
    ADD_LIBRARY(pcan PlxCan.cpp PlxCan.h)
  ELSE (inc_plxapi AND lib_plxapi)
    MESSAGE(SEND_ERROR "Cannot find plxapi lib, check plxCanApi installation instructions")
  ENDIF (inc_plxapi AND lib_plxapi)
ENDIF (NOT SKIP_pcan)
