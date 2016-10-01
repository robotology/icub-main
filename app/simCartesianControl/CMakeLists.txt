# Copyright: 2010 RobotCub Consortium
# Author: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

set(appname simCartesianControl)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB xml ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.xml)

yarp_install(FILES ${scripts} DESTINATION ${ICUB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
yarp_install(FILES ${conf} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${xml} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(DIRECTORY conf/cartesian DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname})

