# Copyright: (C) 2013 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME coman_handler_lib)
PROJECT(${PROJECTNAME})

if(ICUB_COMPILE_COMAN_LIBRARY )

#message(INFO " compiling robolli")
	## Look for robolli lib
    find_package(Robolli)
    if(NOT ROBOLLI_FOUND)
        message(ERROR " Robolli lib was not found!!! Disable coman modules")
        return()      # if robolli is not found cannot continue
    endif(NOT ROBOLLI_FOUND)

	# #add comanDeviceHandler to the group
	set(handler_SOURCES     comanDevicesHandler.cpp)
	set(handler_HEADERS     comanDevicesHandler.hpp)
	
	include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../debugStream)
	include_directories( ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}../ ${robolli_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS}  ${iCubDev_INCLUDE_DIRS})

	set(handler_SOURCES     comanDevicesHandler.cpp)
	set(handler_HEADERS     comanDevicesHandler.hpp)
	add_library(${PROJECTNAME} ${handler_SOURCES} ${handler_HEADERS})
        set_target_properties(${PROJECTNAME} PROPERTIES COMPILE_FLAGS "-std=c++0x")
        target_link_libraries(${PROJECTNAME} ${robolli_LIB} rt debugStream yaml-cpp pthread)
	icub_export_library(${PROJECTNAME} )

#else()
#	message(INFO " skipping robolli")
endif(ICUB_COMPILE_COMAN_LIBRARY)
