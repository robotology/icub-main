# Copyright: (C) 2010 RobotCub Consortium
# Authors: Giorgio Metta
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# Based on code by Paul Fitzpatrick & Lorenzo Natale

# Cmake code for adding a module to the iCub main build.
# You can assume yarp has been already found and is 
# available.

SET(PROJECTNAME logpolarConvertExample)

PROJECT(${PROJECTNAME})
FILE(GLOB folder_source src/*.cpp)
#FILE(GLOB folder_header include/iCub/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
#SOURCE_GROUP("Header Files" FILES ${folder_header})

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${logpolar_INCLUDE_DIRS})
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${iCubDev_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECTNAME} logpolar ${YARP_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)

########## application
set(appname logpolarConvertExample)
icub_app(${appname})
	
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

icub_app_install(${appname} 
				FILES ${conf}
				DESTINATION conf/)
			
icub_add_target(${appname})




