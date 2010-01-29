IF (COMPILE_DEVICE_LIBRARY)
  PREPARE_DEVICE(canmotioncontrol TYPE CanBusMotionControl
    INCLUDE CanBusMotionControl.h 
    WRAPPER controlboard)
ENDIF (COMPILE_DEVICE_LIBRARY)

IF (NOT SKIP_canmotioncontrol)
  FIND_PACKAGE(YARP REQUIRED)
  FIND_PACKAGE(ICUBDEV REQUIRED) 
  INCLUDE_DIRECTORIES(${ICUBDEV_INCLUDE_DIRS})
    
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../motionControlLib/)

  # LINK_LIBRARIES(${CMAKE_CURRENT_SOURCE_DIR}/../motionControlLib/libmotionControl.a)
  #  SET(MOTIONCONTROL_LIBRARIES motionControl)
  #  LINK_LIBRARIES(${MOTIONCONTROL_LIBRARIES})

  ADD_LIBRARY(canmotioncontrol CanBusMotionControl.cpp CanBusMotionControl.h ThreadTable2.cpp ThreadPool2.cpp)
  TARGET_LINK_LIBRARY(canmotioncontrol ${ICUBDEV_LIBRARIES})
ENDIF (NOT SKIP_canmotioncontrol)
