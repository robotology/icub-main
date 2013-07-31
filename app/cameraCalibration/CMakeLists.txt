# Copyright: 2010 RobotCub Consortium
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname cameraCalibration)

icub_app(${appname})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)
file(GLOB data ${CMAKE_CURRENT_SOURCE_DIR}/data/*.pdf)

icub_app_install(${appname} FILES ${conf})
icub_app_install(${appname} FILES ${scripts} DESTINATION scripts)
icub_app_install(${appname} FILES ${data} DESTINATION data)

yarp_install(FILES ${conf} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${scripts} DESTINATION ${ICUB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
yarp_install(FILES ${data} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname}/data)


