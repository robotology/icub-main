set(appname pf3dBottomup)

icub_app(${appname})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

file(GLOB models ${CMAKE_CURRENT_SOURCE_DIR}/conf/models/*.bmp)

icub_app_install(${appname}
				FILES ${conf}
				DESTINATION conf
				)
				
icub_app_install(${appname} 
				FILES ${models}
				DESTINATION conf/models
				)

