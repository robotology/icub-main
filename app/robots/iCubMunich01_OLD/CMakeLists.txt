set(appname iCubMunich01)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)
set(firmware ${CMAKE_CURRENT_SOURCE_DIR}/scripts/firmwareUpdate.txt)
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

yarp_install(FILES ${scripts} DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname}/scripts)
yarp_install(FILES ${firmware} DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname}/scripts)
yarp_install(FILES ${conf} DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname})
yarp_install(DIRECTORY conf/camera DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname} PATTERN .svn EXCLUDE) 


