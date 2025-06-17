# Copyright: (C) 2024 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Jacopo Losi
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname fineCalibrationChecker)
file(GLOB 
    conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini
    utl ${CMAKE_CURRENT_SOURCE_DIR}/*.csv
)
yarp_install(FILES ${conf} ${utl} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname})
