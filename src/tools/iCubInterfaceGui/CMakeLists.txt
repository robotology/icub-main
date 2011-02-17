#
# Copyright (C) 2010 RobotCub Consortium
# Author: Alessandro Scalzo alessandro.scalzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(iCubInterfaceGui)

if(ICUB_USE_GtkMM)
    add_subdirectory(iCubInterfaceGuiClient)
else(ICUB_USE_GtkMM)
    message(STATUS "GtkMM not selected, skipping iCubInterfaceGuiClient")
endif(ICUB_USE_GtkMM)

add_subdirectory(iCubInterfaceGuiLib)
#add_subdirectory(iCubInterfaceGuiServer)
