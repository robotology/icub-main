set(appname simConfig)

icub_app(${appname})

file(GLOB init ${CMAKE_CURRENT_SOURCE_DIR}/*.ini)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB data ${CMAKE_CURRENT_SOURCE_DIR}/data/*.bmp)
file(GLOB data/model ${CMAKE_CURRENT_SOURCE_DIR}/data/model/*)
file(GLOB data/texture ${CMAKE_CURRENT_SOURCE_DIR}/data/texture/*.raw)
file(GLOB data/texture/skybox ${CMAKE_CURRENT_SOURCE_DIR}/data/texture/skybox/*.raw)
file(GLOB models ${CMAKE_CURRENT_SOURCE_DIR}/models/*)


icub_app_install(${appname} 
				FILES ${init})

icub_app_install(${appname} 
				FILES ${scripts}
				DESTINATION scripts)

icub_app_install(${appname} 
				FILES ${conf}
				DESTINATION conf)

icub_app_install(${appname} 
				FILES ${data}
				DESTINATION data)

icub_app_install(${appname} 
				FILES ${data/model}
				DESTINATION data/model)

icub_app_install(${appname} 
				FILES ${data/texture}
				DESTINATION data/texture)

icub_app_install(${appname} 
				FILES ${data/texture/skybox}
				DESTINATION data/texture/skybox)

icub_app_install(${appname} 
				FILES ${models}
				DESTINATION models)

