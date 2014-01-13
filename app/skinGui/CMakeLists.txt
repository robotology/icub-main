set(appname skinGui)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.glade ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.txt)
file(GLOB confpos ${CMAKE_CURRENT_SOURCE_DIR}/conf/positions/*.txt)
file(GLOB confgui ${CMAKE_CURRENT_SOURCE_DIR}/conf/skinGui/*.ini)

yarp_install(FILES ${conf} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${confpos} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname}/positions)
yarp_install(FILES ${confgui} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname}/skinGui)
yarp_install(FILES ${scripts} DESTINATION ${ICUB_APPLICATIONS_TEMPLATES_INSTALL_DIR})

