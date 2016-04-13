set(appname iCubLesVillersLesNancy01)

set(scripts icub_all.xml robotInterface.ini general.xml iKinGazeCtrl.ini icubEyes.ini pf3dTracker.ini cartesianSolver.ini)

yarp_install(FILES ${scripts} DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${dragonFly} DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname})
yarp_install(DIRECTORY calibrators DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname} PATTERN .svn EXCLUDE)  
yarp_install(DIRECTORY cartesian DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname} PATTERN .svn EXCLUDE)
yarp_install(DIRECTORY wrappers DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname} PATTERN .svn EXCLUDE)
yarp_install(DIRECTORY hardware DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname} PATTERN .svn EXCLUDE) 
yarp_install(DIRECTORY camera DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname} PATTERN .svn EXCLUDE) 


