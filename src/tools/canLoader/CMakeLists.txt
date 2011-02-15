# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PROJECT(canLoader)

if(ICUB_USE_GtkPlus)
	ADD_SUBDIRECTORY(canLoader)
else(ICUB_USE_GtkPlus)
	message(STATUS "GtkPlus not selected, skipping canLoader")
endif(ICUB_USE_GtkPlus)

ADD_SUBDIRECTORY(canLoaderLib)

