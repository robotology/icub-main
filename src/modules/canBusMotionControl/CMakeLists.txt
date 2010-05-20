IF (COMPILE_DEVICE_LIBRARY)
  PREPARE_DEVICE(canmotioncontrol TYPE CanBusMotionControl
    INCLUDE CanBusMotionControl.h 
    WRAPPER controlboard)
ENDIF (COMPILE_DEVICE_LIBRARY)

IF (NOT SKIP_canmotioncontrol)
  FIND_PACKAGE(YARP REQUIRED)
  FIND_PACKAGE(ICUBDEV REQUIRED) 
  FIND_PACKAGE(ACE)
    
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../motionControlLib/
                      ${ACE_INCLUDE_DIRS} ${ICUBDEV_INCLUDE_DIRS})

  SET(server_source CanBusMotionControl.cpp ThreadTable2.cpp ThreadPool2.cpp)
  SET(server_header CanBusMotionControl.h ThreadTable2.h ThreadPool2.h)

  # LINK_LIBRARIES(${CMAKE_CURRENT_SOURCE_DIR}/../motionControlLib/libmotionControl.a)
  #  SET(MOTIONCONTROL_LIBRARIES motionControl)
  #  LINK_LIBRARIES(${MOTIONCONTROL_LIBRARIES})

  ADD_LIBRARY(canmotioncontrol ${server_source} ${server_header})
  TARGET_LINK_LIBRARIES(canmotioncontrol ${ACE_LIBRARIES} ${ICUBDEV_LIBRARIES})
ENDIF (NOT SKIP_canmotioncontrol)
