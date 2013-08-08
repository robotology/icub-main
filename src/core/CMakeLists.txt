# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

option(ICUB_ICUBINTERFACE_EXPERIMENTAL "Experimental: this option may break compilation")
mark_as_advanced(ICUB_ICUBINTERFACE_EXPERIMENTAL)

if((NOT APPLE) OR ICUB_ICUBINTERFACE_EXPERIMENTAL) #compilation enabled by default on windows and linux, experimental on mac
    add_subdirectory(robotInterface)
endif()

add_subdirectory(iCubInterface)
add_subdirectory(emotionInterface)




