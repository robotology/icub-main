set(appname coman)

set(scripts coman.xml robotMotorGui.ini basic.yaml scripts/coman_yarpscope.xml robotInterface.ini general.xml)
yarp_install(FILES ${scripts} DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname})

yarp_install(DIRECTORY wrappers DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname} PATTERN .svn EXCLUDE)
yarp_install(DIRECTORY hardware DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname} PATTERN .svn EXCLUDE)  

