#
# Copyright (C) 2009 RobotCub Consortium
# Author: Alessandro Scalzo alessandro.scalzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

set(appname iCubSkinDemo)

icub_app(${appname})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/*.ini)

icub_app_install(${appname} FILES ${conf})