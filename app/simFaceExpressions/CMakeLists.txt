set(appname simFaceExpressions)

icub_app(${appname})

file(GLOB init ${CMAKE_CURRENT_SOURCE_DIR}/*.ini)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template
                   ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.sh)
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB mask ${CMAKE_CURRENT_SOURCE_DIR}/data/*.ppm)

icub_app_install(${appname} 
				FILES ${init})
				
icub_app_install(${appname} 
				FILES ${scripts}
				DESTINATION scripts)

icub_app_install(${appname} 
				FILES ${conf}
				DESTINATION conf)

icub_app_install(${appname} 
				FILES ${mask}
				DESTINATION mask)				
				




