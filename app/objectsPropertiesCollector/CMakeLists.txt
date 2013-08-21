# Copyright: 2010 RobotCub Consortium
# Author: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

set(appname objectsPropertiesCollector)

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

if (__ICUB_INSTALL_APPLICATIONS__)
icub_app(${appname})
icub_app_install(${appname} FILES ${conf})
endif()

yarp_install(FILES ${conf} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname})

                           
