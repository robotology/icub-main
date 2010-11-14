set(appname iCubSkinDemo)

icub_app(${appname})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/*.ini)

icub_app_install(${appname} 
				FILES ${conf})			

