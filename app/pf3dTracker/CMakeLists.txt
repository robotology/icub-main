set(appname pf3dTracker)

icub_app(${appname})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini
                ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.txt
				${CMAKE_CURRENT_SOURCE_DIR}/conf/*.csv)

file(GLOB models ${CMAKE_CURRENT_SOURCE_DIR}/conf/models/*.bmp
		  		  ${CMAKE_CURRENT_SOURCE_DIR}/conf/models/*.csv)

icub_app_install(${appname} 
				FILES ${conf}
				DESTINATION conf
				)
				
icub_app_install(${appname} 
				FILES ${models}
				DESTINATION conf/models
				)

icub_add_target(${appname})

