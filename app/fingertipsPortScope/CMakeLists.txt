# Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Francesco Giovannini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname fingertipsPortScope)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)
yarp_install(FILES ${scripts} DESTINATION ${ICUB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
