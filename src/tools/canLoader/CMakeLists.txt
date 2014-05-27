# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PROJECT(canLoader)

ADD_SUBDIRECTORY(canLoaderLib)

if(ICUB_USE_GTK2)
    ADD_SUBDIRECTORY(canLoader)
else(ICUB_USE_GTK2)
    message(STATUS "GTK2 not selected, skipping canLoader")
endif(ICUB_USE_GTK2)



