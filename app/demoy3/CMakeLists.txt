set(appname demoy3)

icub_app(${appname})

file(GLOB positions ${CMAKE_CURRENT_SOURCE_DIR}/*.*)
file(GLOB crf-files ${CMAKE_CURRENT_SOURCE_DIR}/crf/*.*)
file(GLOB hannover-files ${CMAKE_CURRENT_SOURCE_DIR}/hannover/*.*)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml.template)

icub_app_install(${appname} FILES ${positions})
icub_app_install(${appname} FILES ${crf-files} DESTINATION crf)
icub_app_install(${appname} FILES ${hannover-files} DESTINATION hannover)
icub_app_install(${appname} FILES ${scripts} DESTINATION scripts)

yarp_install(FILES ${positions} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${crf-files} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname}/crf)
yarp_install(FILES ${hannover-files} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname}/hannover)
yarp_install(FILES ${scripts} DESTINATION ${ICUB_APPLICATIONS_TEMPLATES_INSTALL_DIR})


