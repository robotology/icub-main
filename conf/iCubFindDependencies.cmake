# Copyright: (C) 2009 RobotCub Consortium
# Authos: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# Based on code from yarp.

message(STATUS "Detecting required libraries")

find_package(YARP)

message(STATUS "I have found the following libraries:")

if (YARP_FOUND)
  set(ICUB_HAS_YARP true)
  message(STATUS "found YARP")
endif (YARP_FOUND)

