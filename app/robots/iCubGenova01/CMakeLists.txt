set(appname iCubGenova01)

icub_app(${appname})

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)
file(GLOB firmware ${CMAKE_CURRENT_SOURCE_DIR}/scripts/firmwareUpdate.txt)
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB crawling_conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/crawling/*.ini)

icub_app_install(${appname} 
				FILES ${scripts}
				DESTINATION scripts)
icub_app_install(${appname} 
				FILES ${firmware}
				DESTINATION scripts)
icub_app_install(${appname} 
				FILES ${conf}
				DESTINATION conf)						
icub_app_install(${appname} 
				FILES ${crawling_conf}
				DESTINATION conf/crawling)	



