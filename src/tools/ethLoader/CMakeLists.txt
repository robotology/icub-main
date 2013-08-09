# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale, Alessandro Scalzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PROJECT(ethLoader)

if(ICUB_USE_GTK2)
	ADD_SUBDIRECTORY(ethLoader)
else(ICUB_USE_GTK2)
	message(STATUS "GTK2 not selected, skipping ethLoader")
endif(ICUB_USE_GTK2)

ADD_SUBDIRECTORY(ethLoaderLib)

