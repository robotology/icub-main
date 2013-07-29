set(appname iCubGenova02)

icub_app(${appname})

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB crawling_conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/crawling/*.ini)

icub_app_install(${appname} 
				FILES ${scripts}
				DESTINATION scripts)

icub_app_install(${appname} 
				FILES ${conf}
				DESTINATION conf)		

icub_app_install(${appname} 
				FILES ${crawling_conf}
				DESTINATION conf/crawling)

yarp_install(FILES ${scripts} DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname}/scripts)	
yarp_install(FILES ${conf} DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${crawling_conf} DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname}/crawling)				


