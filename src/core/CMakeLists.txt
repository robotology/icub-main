# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

option(ICUB_ICUBINTERFACE_EXPERIMENTAL "Experimental: this option may break compilation")
mark_as_advanced(ICUB_ICUBINTERFACE_EXPERIMENTAL)

if (ICUB_ICUBINTERFACE_EXPERIMENTAL)
	add_subdirectory(iCubInterface-branch)
else()
	add_subdirectory(iCubInterface)
endif()
	
add_subdirectory(emotionInterface)




