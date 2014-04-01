# Copyright: (C) 2008 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


IF (ICUB_CONTROLWRAPPER_WIP)
   PREPARE_DEVICE(canmotioncontrol TYPE CanBusMotionControl
		INCLUDE CanBusMotionControlwip.h 
		WRAPPER controlboard)

   if (ICUB_CANPROTOCOL_STRICT)
       add_definitions(-DICUB_CANPROTOCOL_STRICT)
   endif()

   if (ICUB_CANMASKS_STRICT_FILTER)
       add_definitions(-DICUB_CANMASKS_STRICT_FILTER)
   endif()
   
   if (ICUB_USE_INTERFACEGUI)
       add_definitions(-D_USE_INTERFACEGUI)
   endif (ICUB_USE_INTERFACEGUI)

   ADD_DEFINITIONS(-D_REENTRANT)

   INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../motionControlLib/
                       ${ACE_INCLUDE_DIRS} ${iCubDev_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

   SET(folder_source CanBusMotionControlwip.cpp ThreadTable2.cpp ThreadPool2.cpp)
   SET(folder_header CanBusMotionControlwip.h ThreadTable2.h ThreadPool2.h)

   SOURCE_GROUP("Source Files" FILES ${folder_source})
   SOURCE_GROUP("Header Files" FILES ${folder_header})

   yarp_add_plugin(canmotioncontrol ${folder_source} ${folder_header})
   TARGET_LINK_LIBRARIES(canmotioncontrol ${ACE_LIBRARIES} iCubDev ${YARP_LIBRARIES})
  
   icub_export_library(canmotioncontrol)
else()
   PREPARE_DEVICE(canmotioncontrol TYPE CanBusMotionControl
		INCLUDE CanBusMotionControl.h 
		WRAPPER controlboard)

   if (ICUB_CANPROTOCOL_STRICT)
       add_definitions(-DICUB_CANPROTOCOL_STRICT)
   endif()

   if (ICUB_CANMASKS_STRICT_FILTER)
       add_definitions(-DICUB_CANMASKS_STRICT_FILTER)
   endif()

   if (ICUB_USE_INTERFACEGUI)
       add_definitions(-D_USE_INTERFACEGUI)
   endif (ICUB_USE_INTERFACEGUI)

   ADD_DEFINITIONS(-D_REENTRANT)

   INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../motionControlLib/
                       ${ACE_INCLUDE_DIRS} ${iCubDev_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

   SET(folder_source CanBusMotionControl.cpp ThreadTable2.cpp ThreadPool2.cpp)
   SET(folder_header CanBusMotionControl.h ThreadTable2.h ThreadPool2.h)

   SOURCE_GROUP("Source Files" FILES ${folder_source})
   SOURCE_GROUP("Header Files" FILES ${folder_header})

   yarp_add_plugin(canmotioncontrol ${folder_source} ${folder_header})
   TARGET_LINK_LIBRARIES(canmotioncontrol ${ACE_LIBRARIES} iCubDev ${YARP_LIBRARIES})
  
   icub_export_library(canmotioncontrol)
ENDIF () 




