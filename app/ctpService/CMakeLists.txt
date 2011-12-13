# Copyright: 2010 RobotCub Consortium
# Author: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

set(appname ctpService)

icub_app(${appname})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.*)

icub_app_install(${appname} 
				FILES ${conf}
				DESTINATION conf)			
				
