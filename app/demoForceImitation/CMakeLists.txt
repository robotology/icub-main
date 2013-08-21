set(appname demoForceImitation)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)
if (__ICUB_INSTALL_APPLICATIONS__)
icub_app(${appname})
icub_app_install(${appname} FILES ${scripts} DESTINATION scripts)
endif()
yarp_install(FILES ${scripts} DESTINATION ${ICUB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
