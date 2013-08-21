set(appname iCubParis01)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)
file(GLOB firmware ${CMAKE_CURRENT_SOURCE_DIR}/scripts/firmwareUpdate.txt)
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB crawling_conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/crawling/*.ini)
file(GLOB balancing_conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/balancing/*.ini)

if (__ICUB_INSTALL_APPLICATIONS__)
icub_app(${appname})
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
		
icub_app_install(${appname} 
				FILES ${balancing_conf}
				DESTINATION conf/balancing)	
endif()

yarp_install(FILES ${scripts} DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname}/scripts)	
yarp_install(FILES ${conf} DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname})				
yarp_install(FILES ${crawling_conf} DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname}/crawling)	
yarp_install(FILES ${balancing_conf} DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname}/balancing)	

