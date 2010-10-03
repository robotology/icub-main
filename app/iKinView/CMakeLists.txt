set(appname iKinView)

icub_app(${appname})

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.m)

icub_app_install(${appname} 
				FILES ${scripts}
				DESTINATION scripts)	
				
icub_add_target(${appname})



