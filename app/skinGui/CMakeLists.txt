set(appname skinGui)

icub_app(${appname})

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.glade ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.txt)
file(GLOB confpos ${CMAKE_CURRENT_SOURCE_DIR}/conf/positions/*.txt)
file(GLOB confgui ${CMAKE_CURRENT_SOURCE_DIR}/conf/skinGui/*.ini)

icub_app_install(${appname} 
				FILES ${scripts}
				DESTINATION scripts)

icub_app_install(${appname} 
				FILES ${conf}
				DESTINATION conf)

icub_app_install(${appname} 
				FILES ${confpos}
				DESTINATION conf/positions)

icub_app_install(${appname} 
				FILES ${confgui}
				DESTINATION conf/skinGui)
				
#icub_add_target(${appname})

yarp_install(FILES ${conf} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname}/conf )
yarp_install(FILES ${confpos} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname}/conf/positions )
yarp_install(FILES ${confgui} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname}/conf/skinGui )
yarp_install(FILES ${scripts}
                DESTINATION ${ICUB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
