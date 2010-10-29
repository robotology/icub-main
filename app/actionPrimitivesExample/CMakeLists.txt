set(appname actionPrimitivesExample)

icub_app(${appname})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

icub_app_install(${appname}
				FILES ${conf}
				DESTINATION conf)			
				
icub_add_target(${appname})

