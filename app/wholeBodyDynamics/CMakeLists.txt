set(appname wholeBodyDynamics)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

if (__ICUB_INSTALL_APPLICATIONS__)
icub_app(${appname})
icub_app_install(${appname} FILES ${scripts} DESTINATION scripts)
icub_app_install(${appname} FILES ${conf})
endif()
                           
yarp_install(FILES ${conf} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${scripts} DESTINATION ${ICUB_APPLICATIONS_TEMPLATES_INSTALL_DIR})

