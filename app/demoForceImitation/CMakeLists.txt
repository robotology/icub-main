set(appname demoForceImitation)
icub_app(${appname})

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)
icub_app_install(${appname} 
				FILES ${scripts}
				DESTINATION scripts)

yarp_install(FILES ${scripts}
                DESTINATION ${ICUB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
