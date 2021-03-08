#
# Copyright (C) 2007 RobotCub Consortium
# Author: Alessandro Scalzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

yarp_prepare_plugin(dragonfly2
                    CATEGORY device
                    TYPE yarp::dev::DragonflyDeviceDriver2Rgb
                    INCLUDE common/DragonflyDeviceDriver2.h
                    EXTRA_CONFIG WRAPPER=grabberDual)

# dragonfly2raw is always enabled when dragonfly2 is enabled
# This is a workaround to enable the creation of the additional source files and to hide the entry in ccmake
set(ENABLE_icubmod_dragonfly2raw ${ENABLE_icubmod_dragonfly2} CACHE INTERNAL "Enable/disable compilation of icubmod_dragonfly2raw")
yarp_prepare_plugin(dragonfly2raw
                    CATEGORY device
                    TYPE yarp::dev::DragonflyDeviceDriver2Raw
                    INCLUDE common/DragonflyDeviceDriver2.h
                    EXTRA_CONFIG WRAPPER=grabberDual)

if(NOT SKIP_dragonfly2)
  if(WIN32)
    find_package(DRAGONFLYAPI)
  else()
    # On Linux we do not need any dragonfly-specific API, just libdc1394
    find_package(Libdc1394)
    set(DRAGONFLYAPI_FOUND ${Libdc1394_FOUND})
    set(DRAGONFLYAPI_INC_DIRS ${Libdc1394_INCLUDE_DIRS})
    set(DRAGONFLYAPI_LIB ${Libdc1394_LIBRARIES})
  endif()

  set(EXTRA_SOURCES)
  if(NOT DRAGONFLYAPI_FOUND)
	  message(SEND_ERROR "dragonfly2: dragonfly api not found, turn off device")
  else()
    add_definitions(-D_REENTRANT)

    include_directories(${DRAGONFLYAPI_INC_DIRS})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/common)
    if(WIN32)
	    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/winnt)
	    set(EXTRA_SOURCES winnt/FirewireCameraDC1394-DR2_2.h winnt/FirewireCameraDC1394-DR2_2.cpp)
    else()
	    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/linux)
	    set(EXTRA_SOURCES linux/FirewireCameraDC1394-DR2_2.h linux/FirewireCameraDC1394-DR2_2.cpp)
    endif()

    yarp_add_plugin(dragonfly2 common/DragonflyDeviceDriver2.h common/DragonflyDeviceDriver2.cpp ${EXTRA_SOURCES})
    target_link_libraries(dragonfly2 ${YARP_LIBRARIES} ${DRAGONFLYAPI_LIB})
    icub_export_plugin(dragonfly2)
  
    yarp_install(TARGETS dragonfly2
                 COMPONENT Runtime
                 LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})
  endif()
endif()

    
