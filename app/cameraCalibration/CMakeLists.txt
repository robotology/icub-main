# Copyright: 2010 RobotCub Consortium
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname cameraCalibration)

icub_app(${appname})

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

icub_app_install(${appname} 
				FILES ${scripts}
				DESTINATION scripts)

icub_app_install(${appname} 
				FILES ${CMAKE_CURRENT_SOURCE_DIR}/README)

icub_app_install(${appname} 
				FILES ${conf}
				DESTINATION conf)


