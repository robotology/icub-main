set(appname coman)

icub_app(${appname})

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)
file(GLOB conf    ${CMAKE_CURRENT_SOURCE_DIR}/conf/*)
file(GLOB conf_hw ${CMAKE_CURRENT_SOURCE_DIR}/conf/hardware/*)
file(GLOB conf_wr ${CMAKE_CURRENT_SOURCE_DIR}/conf/wrappers/*)

icub_app_install(${appname} 
				FILES ${scripts}
				DESTINATION scripts)

icub_app_install(${appname} 
				FILES ${conf}
				DESTINATION conf)			
    
icub_app_install(${appname} 
                                FILES ${conf_hw}
                                DESTINATION conf/hardware)                       
    
icub_app_install(${appname} 
                                FILES ${conf_wr}
                                DESTINATION conf/wrappers)                       


