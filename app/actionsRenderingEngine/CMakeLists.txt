# Copyright: 2011 RobotCub Consortium
# Author: Carlo Ciliberto
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

set(appname actionsRenderingEngine)

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB actions_left ${CMAKE_CURRENT_SOURCE_DIR}/actions/left/*.action)
file(GLOB actions_right ${CMAKE_CURRENT_SOURCE_DIR}/actions/right/*.action)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)

if (__ICUB_INSTALL_APPLICATIONS__)
icub_app(${appname})
icub_app_install(${appname} FILES ${conf})
icub_app_install(${appname} FILES ${actions_left} DESTINATION actions/left/)
icub_app_install(${appname} FILES ${actions_right} DESTINATION actions/right/)
icub_app_install(${appname} FILES ${scripts} DESTINATION scripts/)
endif()
yarp_install(FILES ${conf} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${actions_left} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname}/actions/left)
yarp_install(FILES ${actions_right} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname}/actions/right)
yarp_install(FILES ${scripts} DESTINATION ${ICUB_APPLICATIONS_TEMPLATES_INSTALL_DIR})

