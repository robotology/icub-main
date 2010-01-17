IF (COMPILE_DEVICE_LIBRARY)
  PREPARE_DEVICE(canmotioncontrolxp TYPE CanBusMotionControl
    INCLUDE CanBusMotionControl2.h 
    WRAPPER controlboard)
ENDIF (COMPILE_DEVICE_LIBRARY)

IF (NOT SKIP_canmotioncontrolxp)
  FIND_PACKAGE(YARP REQUIRED)
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../motionControlLib/)

  # LINK_LIBRARIES(${CMAKE_CURRENT_SOURCE_DIR}/../motionControlLib/libmotionControl.a)
  #  SET(MOTIONCONTROL_LIBRARIES motionControl)
  #  LINK_LIBRARIES(${MOTIONCONTROL_LIBRARIES})

  FIND_PACKAGE(ICUBDEV REQUIRED)
  INCLUDE_DIRECTORIES(${ICUBDEV_INCLUDE_DIRS})
  
  ADD_LIBRARY(canmotioncontrolxp CanBusMotionControl2.cpp CanBusMotionControl2.h ThreadTable2.cpp ThreadPool2.cpp)

  TARGET_LINK_LIBRARIES(canmotioncontrolxp ${ICUBDEV_LIBRARIES})
  
ENDIF (NOT SKIP_canmotioncontrolxp)
