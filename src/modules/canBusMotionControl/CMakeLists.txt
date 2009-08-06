IF (COMPILE_DEVICE_LIBRARY)
  PREPARE_DEVICE(canmotioncontrol TYPE CanBusMotionControl
    INCLUDE CanBusMotionControl.h 
    WRAPPER controlboard)
ENDIF (COMPILE_DEVICE_LIBRARY)

IF (NOT SKIP_canmotioncontrol)
  FIND_PACKAGE(YARP REQUIRED)
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../motionControlLib/)

  # LINK_LIBRARIES(${CMAKE_CURRENT_SOURCE_DIR}/../motionControlLib/libmotionControl.a)
  #  SET(MOTIONCONTROL_LIBRARIES motionControl)
  #  LINK_LIBRARIES(${MOTIONCONTROL_LIBRARIES})

  ADD_LIBRARY(canmotioncontrol CanBusMotionControl.cpp CanBusMotionControl.h ThreadTable2.cpp ThreadPool2.cpp)
ENDIF (NOT SKIP_canmotioncontrol)
