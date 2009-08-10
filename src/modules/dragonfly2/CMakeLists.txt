IF (COMPILE_DEVICE_LIBRARY)
  PREPARE_DEVICE(dragonfly2 TYPE DragonflyDeviceDriver2 INCLUDE DragonflyDeviceDriver2.h WRAPPER grabber)
ENDIF (COMPILE_DEVICE_LIBRARY)

IF(NOT SKIP_dragonfly2)
  FIND_PACKAGE(YARP REQUIRED)

  SET (OK FALSE)
  SET (EXTRA_SOURCES)

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
	SET(SYS linux)
	FIND_LIBRARY(lib_raw1394 raw1394)
	FIND_LIBRARY(lib_dc1394 dc1394)
	IF (lib_raw1394 AND lib_dc1394)
      SET (OK TRUE)
      LINK_LIBRARIES(${lib_raw1394} ${lib_dc1394})
	  SET(EXTRA_SOURCES linux/FirewireCameraSet2.cpp linux/FirewireCameraSet2.h linux/FirewireCameraDC1394-DR2_2.h)
	ENDIF (lib_raw1394 AND lib_dc1394)
  ENDIF (WIN32)

  IF (OK)

	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/common)
	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/${SYS})

	ADD_LIBRARY(dragonfly2 ${SYS}/DragonflyDeviceDriver2.cpp  common/DragonflyDeviceDriver2.h ${EXTRA_SOURCES})  

  ELSE (OK)
	MESSAGE(SEND_ERROR "dragonfly2 module: missing libraries. Install them or disable module from the list.")
  ENDIF (OK)

ENDIF(NOT SKIP_dragonfly2)
