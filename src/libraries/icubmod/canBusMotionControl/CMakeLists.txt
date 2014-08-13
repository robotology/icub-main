# Copyright: (C) 2008 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


PREPARE_DEVICE(canmotioncontrol TYPE CanBusMotionControl
    INCLUDE CanBusMotionControl.h
    WRAPPER controlboardwrapper2)

if (ENABLE_icubmod_canmotioncontrol)

if(ICUB_HAS_icub_firmware_shared)
    set(FIRMWARE_SHARED_INC         ${icub_firmware_shared_canProtocolLib_INCLUDE_DIR})


   if (ICUB_CANPROTOCOL_STRICT)
       add_definitions(-DICUB_CANPROTOCOL_STRICT)
   endif()

   if (ICUB_AUTOMATIC_MODE_SWITCHING)
	add_definitions(-DICUB_AUTOMATIC_MODE_SWITCHING)
   endif()

   if (ICUB_CANMASKS_STRICT_FILTER)
       add_definitions(-DICUB_CANMASKS_STRICT_FILTER)
   endif()

   if (ICUB_USE_INTERFACEGUI)
       add_definitions(-D_USE_INTERFACEGUI)
   endif (ICUB_USE_INTERFACEGUI)

   ADD_DEFINITIONS(-D_REENTRANT)

   INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../motionControlLib/
                       ${ACE_INCLUDE_DIRS} ${iCubDev_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS}
                       ${FIRMWARE_SHARED_INC}
                       ${CMAKE_SOURCE_DIR}/src/libraries/icubmod/debugStream/
                       )

   SET(folder_source CanBusMotionControl.cpp ThreadTable2.cpp ThreadPool2.cpp)
   SET(folder_header CanBusMotionControl.h ThreadTable2.h ThreadPool2.h)

   SOURCE_GROUP("Source Files" FILES ${folder_source})
   SOURCE_GROUP("Header Files" FILES ${folder_header})

   yarp_add_plugin(canmotioncontrol ${folder_source} ${folder_header})
   TARGET_LINK_LIBRARIES(canmotioncontrol ${ACE_LIBRARIES} iCubDev ${YARP_LIBRARIES} debugStream)

   icub_export_library(canmotioncontrol)

else(ICUB_HAS_icub_firmware_shared)
  message(ERROR " canBusMotionControl: cannot find icub_firmware_shared library, turn off device
  embObj library can now be found in the icub-firmware-shared package that
  can be downloaded from here:
  .
       https://github.com/robotology/icub-firmware-shared
  .    ")
endif(ICUB_HAS_icub_firmware_shared)

endif (ENABLE_icubmod_canmotioncontrol)


