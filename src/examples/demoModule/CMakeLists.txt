# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

#
# demoModule CMake file.
#
set(PROJECTNAME "demoModule")
project(${PROJECTNAME})

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${YARP_INCLUDE_DIRS} 	
)

# Search for source code.
file(GLOB folder_source src/*.cpp src/*.cc src/*.c)
file(GLOB folder_header include/iCub/*.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

add_executable(${PROJECTNAME} 
	${folder_source} 
	${folder_header}
)

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES}) 

install(TARGETS ${PROJECTNAME} DESTINATION bin)

########## application
set(appname demo)

icub_app(${appname})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

icub_app_install(${appname}
				FILES ${conf}
				DESTINATION conf)			
