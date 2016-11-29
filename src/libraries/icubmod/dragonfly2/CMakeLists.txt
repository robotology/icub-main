#
# Copyright (C) 2007 RobotCub Consortium
# Author: Alessandro Scalzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#


yarp_prepare_plugin(dragonfly2 CATEGORY device TYPE yarp::dev::DragonflyDeviceDriver2Rgb INCLUDE DragonflyDeviceDriver2.h EXTRA_CONFIG WRAPPER=grabber)

# dragonfly2raw is always enabled when dragonfly2 is enabled
# This is a workaround to enable the creation of the additional source files and to hide the entry in ccmake
set(ENABLE_icubmod_dragonfly2raw ${ENABLE_icubmod_dragonfly2} CACHE INTERNAL "Enable/disable compilation of icubmod_dragonfly2raw")
yarp_prepare_plugin(dragonfly2raw  CATEGORY device TYPE yarp::dev::DragonflyDeviceDriver2Raw INCLUDE DragonflyDeviceDriver2.h EXTRA_CONFIG WRAPPER=grabber)

IF(NOT SKIP_dragonfly2)
  FIND_PACKAGE(DRAGONFLYAPI)
  
  SET (EXTRA_SOURCES)
  IF(NOT DRAGONFLYAPI_FOUND)
	MESSAGE(SEND_ERROR "dragonfly2: dragonfly api not found, turn off device")
  ELSE(NOT DRAGONFLYAPI_FOUND)

  ADD_DEFINITIONS(-D_REENTRANT)

  if(WIN32)
	INCLUDE_DIRECTORIES(${DRAGONFLYAPI_INC_DIRS})
	INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})
	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/common)
	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/winnt)
	SET(EXTRA_SOURCES winnt/FirewireCameraDC1394-DR2_2.h winnt/FirewireCameraDC1394-DR2_2.cpp)
  else()
    INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/common)
	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/linux)
	INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})
	SET(EXTRA_SOURCES linux/FirewireCameraDC1394-DR2_2.h linux/FirewireCameraDC1394-DR2_2.cpp)
  endif()

  yarp_add_plugin(dragonfly2 common/DragonflyDeviceDriver2.h common/DragonflyDeviceDriver2.cpp ${EXTRA_SOURCES})
  target_link_libraries(dragonfly2 ${YARP_LIBRARIES} ${DRAGONFLYAPI_LIB})
  icub_export_plugin(dragonfly2)
  yarp_install(FILES dragonfly2.ini dragonfly2raw.ini DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

  if(NOT WIN32)
    add_executable(firewirebusreset linux/fwBusReset_main.cpp ${EXTRA_SOURCES})
    target_link_libraries(firewirebusreset ${YARP_LIBRARIES} ${DRAGONFLYAPI_LIB})
    install(TARGETS firewirebusreset DESTINATION bin) 
  endif()

  ENDIF(NOT DRAGONFLYAPI_FOUND)
ENDIF(NOT SKIP_dragonfly2)

    
