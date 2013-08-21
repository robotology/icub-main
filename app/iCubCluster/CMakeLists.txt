set(appname iCubCluster)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template ${CMAKE_CURRENT_SOURCE_DIR}/scripts/icub-cluster.py)
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

if (__ICUB_INSTALL_APPLICATIONS__)
icub_app(${appname})
icub_app_install(${appname} FILES ${scripts} DESTINATION scripts FORCE_UPDATE)
icub_app_install(${appname} FILES ${conf} FORCE_UPDATE)
endif()

yarp_install(FILES ${conf} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${scripts} DESTINATION ${ICUB_APPLICATIONS_TEMPLATES_INSTALL_DIR})


