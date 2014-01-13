# Copyright: 2010 RobotCub Consortium
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname cameraCalibration)

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)
file(GLOB data ${CMAKE_CURRENT_SOURCE_DIR}/data/*.pdf)

yarp_install(FILES ${conf} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${scripts} DESTINATION ${ICUB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
yarp_install(FILES ${data} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname}/data)


