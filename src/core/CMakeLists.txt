# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


if(${icub_firmware_shared_FOUND})
    message(STATUS " +++ adding iCubInterface")
    add_subdirectory(iCubInterface)
else()
    message(STATUS " icub-firmware-shared library was not found, disabling iCubInterface")
endif()
add_subdirectory(emotionInterface)




