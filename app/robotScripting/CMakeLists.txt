set(appname robotScripting)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml.template)
file(GLOB example ${CMAKE_CURRENT_SOURCE_DIR}/example/*.sh ${CMAKE_CURRENT_SOURCE_DIR}/example/*.txt)

yarp_install(FILES ${example} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname}/example)
yarp_install(FILES ${scripts} DESTINATION ${ICUB_APPLICATIONS_TEMPLATES_INSTALL_DIR})

