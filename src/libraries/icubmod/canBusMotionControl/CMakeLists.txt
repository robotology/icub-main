PREPARE_DEVICE(canmotioncontrol TYPE CanBusMotionControl
    INCLUDE CanBusMotionControl.h 
    WRAPPER controlboard)

IF (NOT SKIP_canmotioncontrol)
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../motionControlLib/
                      ${ACE_INCLUDE_DIRS} ${iCubDev_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

  SET(server_source CanBusMotionControl.cpp ThreadTable2.cpp ThreadPool2.cpp)
  SET(server_header CanBusMotionControl.h ThreadTable2.h ThreadPool2.h)

  # LINK_LIBRARIES(${CMAKE_CURRENT_SOURCE_DIR}/../motionControlLib/libmotionControl.a)
  #  SET(MOTIONCONTROL_LIBRARIES motionControl)
  #  LINK_LIBRARIES(${MOTIONCONTROL_LIBRARIES})

  ADD_LIBRARY(canmotioncontrol CanBusMotionControl.cpp CanBusMotionControl.h ThreadTable2.cpp ThreadPool2.cpp)
  TARGET_LINK_LIBRARIES(canmotioncontrol ${ACE_LIBRARIES} iCubDev ${YARP_LIBRARIES})
  
  icub_export_library(canmotioncontrol)

  ENDIF (NOT SKIP_canmotioncontrol)
