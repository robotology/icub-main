#
# Copyright (C) 2007 RobotCub Consortium
# Author: Alessandro Scalzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

PREPARE_DEVICE(dragonfly2 TYPE DragonflyDeviceDriver2Rgb INCLUDE DragonflyDeviceDriver2.h WRAPPER grabber)
PREPARE_DEVICE(dragonfly2raw TYPE DragonflyDeviceDriver2Raw INCLUDE DragonflyDeviceDriver2.h WRAPPER grabber)

IF(NOT SKIP_dragonfly2)
  FIND_PACKAGE(DRAGONFLYAPI)
  
  SET (EXTRA_SOURCES)
  IF(NOT DRAGONFLYAPI_FOUND)
	MESSAGE(SEND_ERROR "dragonfly2: dragonfly api not found, turn off device")
  ELSE(NOT DRAGONFLYAPI_FOUND)

  ADD_DEFINITIONS(-D_REENTRANT)
  IF (WIN32)
	INCLUDE_DIRECTORIES(${DRAGONFLYAPI_INC_DIRS})
	INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})
	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/common)
	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/winnt)

    SET(EXTRA_SOURCES winnt/FirewireCameraDC1394-DR2_2.h winnt/FirewireCameraDC1394-DR2_2.cpp)
	yarp_add_plugin(dragonfly2 common/DragonflyDeviceDriver2.h common/DragonflyDeviceDriver2.cpp ${EXTRA_SOURCES})
    target_link_libraries(dragonfly2 ${YARP_LIBRARIES} ${DRAGONFLYAPI_LIB})
  ELSE (WIN32)

	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/common)
	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/linux)
	INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})

	SET(EXTRA_SOURCES linux/FirewireCameraDC1394-DR2_2.h linux/FirewireCameraDC1394-DR2_2.cpp)
	yarp_add_plugin(dragonfly2 common/DragonflyDeviceDriver2.h common/DragonflyDeviceDriver2.cpp ${EXTRA_SOURCES})
  
    target_link_libraries(dragonfly2 ${YARP_LIBRARIES} ${DRAGONFLYAPI_LIB})

    add_executable(firewirebusreset linux/fwBusReset_main.cpp ${EXTRA_SOURCES})
    target_link_libraries(firewirebusreset ${YARP_LIBRARIES} ${DRAGONFLYAPI_LIB})
    install(TARGETS firewirebusreset DESTINATION bin)
     
  ENDIF (WIN32)

  icub_export_library(dragonfly2)
  ENDIF(NOT DRAGONFLYAPI_FOUND)

ENDIF(NOT SKIP_dragonfly2)
