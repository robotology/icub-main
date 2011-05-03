#
# Copyright (C): 2009 The RobotCub Consortium, European Commission FP6 Project IST-004370
# Authors: David Vernon, Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

set(PROJECTNAME logpolarTransform)
project(${PROJECTNAME})

file(GLOB folder_source src/*.cpp src/*.cc src/*.c)
file(GLOB folder_header include/iCub/*.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${logpolar_INCLUDE_DIRS}
	${iCubDev_INCLUDE_DIRS}
    ${YARP_INCLUDE_DIRS}
)

if (folder_source)
	add_executable(${PROJECTNAME} ${folder_source} ${header_files})
	target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} logpolar) 
	install(TARGETS ${PROJECTNAME} DESTINATION bin)
else (folder_source)
    message(STATUS "No source code files found. Please add something")
endif (folder_source)

set(appname logpolarTransform)
icub_app(${appname})
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
icub_app_install(${appname} 
                FILES ${conf}
                DESTINATION conf/)
icub_add_target(${appname})

