set(appname coman)

set(scripts coman.xml coman_lower_body.xml coman_upper_body.xml robotMotorGui.ini basic.yaml robotInterface.ini robolli.xml Readme.txt)
yarp_install(FILES ${scripts} DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname})

yarp_install(DIRECTORY wrappers DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname} PATTERN .svn EXCLUDE)
yarp_install(DIRECTORY hardware DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname} PATTERN .svn EXCLUDE)  
yarp_install(DIRECTORY parts    DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname} PATTERN .svn EXCLUDE)

