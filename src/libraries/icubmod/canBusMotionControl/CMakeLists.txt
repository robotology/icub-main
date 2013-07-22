# Copyright: (C) 2008 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


PREPARE_DEVICE(canmotioncontrol TYPE CanBusMotionControl
		INCLUDE CanBusMotionControl.h 
		WRAPPER controlboard)

PREPARE_DEVICE(canmotioncontrolwip TYPE CanBusMotionControl
		INCLUDE CanBusMotionControlwip.h 
		WRAPPER controlboard)

if (ENABLE_icubmod_canmotioncontrol AND ENABLE_icubmod_canmotioncontrolwip)
	message(FATAL_ERROR "Cannot enable both ENABLE_icubmod_canmotioncontrol and ENABLE_icubmod_canmotioncontrolwip, please choose one")
endif()

	
if (ICUB_CANPROTOCOL_STRICT)
	add_definitions(-DICUB_CANPROTOCOL_STRICT)
endif()

if (ICUB_USE_INTERFACEGUI)
	add_definitions(-D_USE_INTERFACEGUI)
endif (ICUB_USE_INTERFACEGUI)

IF (NOT SKIP_canmotioncontrol)
  ADD_DEFINITIONS(-D_REENTRANT)

  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../motionControlLib/
                      ${ACE_INCLUDE_DIRS} ${iCubDev_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

  
  SET(folder_source CanBusMotionControl.cpp ThreadTable2.cpp ThreadPool2.cpp)
  SET(folder_header CanBusMotionControl.h ThreadTable2.h ThreadPool2.h)
  
  SOURCE_GROUP("Source Files" FILES ${folder_source})
  SOURCE_GROUP("Header Files" FILES ${folder_header})

  ADD_LIBRARY(canmotioncontrol ${folder_source} ${folder_header})
  TARGET_LINK_LIBRARIES(canmotioncontrol ${ACE_LIBRARIES} iCubDev ${YARP_LIBRARIES})
  
  icub_export_library(canmotioncontrol)
ENDIF ()

IF (NOT SKIP_canmotioncontrolwip)
  ADD_DEFINITIONS(-D_REENTRANT)

  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../motionControlLib/
                      ${ACE_INCLUDE_DIRS} ${iCubDev_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

  SET(folder_source CanBusMotionControlwip.cpp ThreadTable2.cpp ThreadPool2.cpp)
  SET(folder_header CanBusMotionControlwip.h ThreadTable2.h ThreadPool2.h)
  
  SOURCE_GROUP("Source Files" FILES ${folder_source})
  SOURCE_GROUP("Header Files" FILES ${folder_header})

  ADD_LIBRARY(canmotioncontrolwip ${folder_source} ${folder_header})
  TARGET_LINK_LIBRARIES(canmotioncontrolwip ${ACE_LIBRARIES} iCubDev ${YARP_LIBRARIES})
  
  icub_export_library(canmotioncontrolwip)
ENDIF ()
