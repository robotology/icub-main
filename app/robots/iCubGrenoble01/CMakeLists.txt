set(appname iCubGrenoble01)

set(scripts icub_all.xml robotMotorGui.ini firmwareUpdate.txt)

yarp_install(FILES ${scripts} DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname})
yarp_install(DIRECTORY calibrators DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname})  
yarp_install(DIRECTORY cartesian DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname})
yarp_install(DIRECTORY wrappers DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname})
yarp_install(DIRECTORY hardware DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname})  
