IF (COMPILE_DEVICE_LIBRARY)
  PREPARE_DEVICE(dragonfly2 TYPE DragonflyDeviceDriver2 INCLUDE DragonflyDeviceDriver2.h WRAPPER grabber)
ENDIF (COMPILE_DEVICE_LIBRARY)

IF(NOT SKIP_dragonfly2)
  FIND_PACKAGE(YARP REQUIRED)
  FIND_PACKAGE(DRAGONFLYAPI)

  SET (EXTRA_SOURCES)

  IF(NOT DRAGONFLYAPI_FOUND)
	MESSAGE(SEND_ERROR "dragonfly2: dragonfly api not found, turn off device")
  ELSE(NOT DRAGONFLYAPI_FOUND)
  IF (WIN32)
	MESSAGE(STATUS "*** CMAKELISTS.TXT for DRAGONFLY not tested on WINDOWS yet")
	MESSAGE(STATUS "*** CMAKELISTS.TXT for DRAGONFLY not tested on WINDOWS yet")
	MESSAGE(STATUS "*** CMAKELISTS.TXT for DRAGONFLY not tested on WINDOWS yet")
	SET(SYS winnt)
	FIND_LIBRARY(lib_dragonfly PGRFlyCapture ${CMAKE_CURRENT_SOURCE_DIR}/../dragonflyApi/winnt/lib)
	IF (lib_dragonfly)
      SET (OK TRUE)
      LINK_LIBRARIES(${lib_dragonfly})

	ENDIF (lib_dragonfly)
	SET(EXTRA_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/../dragonflyApi/winnt/include/PGRFlyCapture.h 
				${CMAKE_CURRENT_SOURCE_DIR}/../dragonflyApi/winnt/include/PGRFlyCapturePlus.h
				${CMAKE_CURRENT_SOURCE_DIR}/../dragonflyApi/winnt/include/pgrerror.h)

	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../dragonflyApi/winnt/include)

  ELSE (WIN32)
	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/common)
	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/linux)

	LINK_LIBRARIES(${DRAGONFLYAPI_LIB})
	SET(EXTRA_SOURCES linux/FirewireCameraSet2.cpp linux/FirewireCameraSet2.h linux/FirewireCameraDC1394-DR2_2.h)
	ADD_LIBRARY(dragonfly2 linux/DragonflyDeviceDriver2.cpp  common/DragonflyDeviceDriver2.h ${EXTRA_SOURCES})  
  ENDIF (WIN32)

  ENDIF(NOT DRAGONFLYAPI_FOUND)

ENDIF(NOT SKIP_dragonfly2)
