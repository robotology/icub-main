set(appname objectsPropertiesCollector)

icub_app(${appname})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

icub_app_install(${appname} 
				FILES ${conf}
				DESTINATION conf)		

				
