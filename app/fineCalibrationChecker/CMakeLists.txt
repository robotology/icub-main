# Copyright: (C) 2024 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Jacopo Losi
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname fineCalibrationChecker)
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
yarp_install(FILES ${conf} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname})
