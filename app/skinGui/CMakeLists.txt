set(appname skinGui)

icub_app(${appname})

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.glade ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.txt)
file(GLOB confpos ${CMAKE_CURRENT_SOURCE_DIR}/conf/positions/*.txt)

icub_app_install(${appname} 
				FILES ${scripts}
				DESTINATION scripts)

icub_app_install(${appname} 
				FILES ${conf}
				DESTINATION conf)

icub_app_install(${appname} 
				FILES ${confpos}
				DESTINATION conf/positions)
				
#icub_add_target(${appname})
