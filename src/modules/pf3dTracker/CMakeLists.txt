# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale, Matteo Taiana, Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PROJECT(pf3dTracker)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(./include)

SUBDIRS(src)



