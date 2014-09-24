# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.



IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    option(ICUB_REALTIME_EXPERIMENTAL "Experimental: Prioritize essential threads to run in the real-time mode" OFF)
    mark_as_advanced (ICUB_REALTIME_EXPERIMENTAL)

    if(ICUB_REALTIME_EXPERIMENTAL)
        add_definitions(-DICUB_USE_REALTIME_LINUX)
    endif(ICUB_REALTIME_EXPERIMENTAL)
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

option(ICUB_ICUBINTERFACE_EXPERIMENTAL "Experimental: this option may break compilation")
mark_as_advanced(ICUB_ICUBINTERFACE_EXPERIMENTAL)

if((NOT APPLE) OR ICUB_ICUBINTERFACE_EXPERIMENTAL) #compilation enabled by default on windows and linux, experimental on mac
    add_subdirectory(robotInterface)
endif()

if(${icub_firmware_shared_FOUND})
    message(STATUS " +++ adding iCubInterface")
    add_subdirectory(iCubInterface)
else()
    message(STATUS " icub-firmware-shared library was not found, disabling iCubInterface")
endif()
add_subdirectory(emotionInterface)




