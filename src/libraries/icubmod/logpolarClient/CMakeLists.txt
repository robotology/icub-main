PREPARE_DEVICE(logpolarclient TYPE ClientLogpolarFrameGrabber
    INCLUDE yarp/dev/ClientLogpolarFrameGrabber.h)

IF(NOT SKIP_logpolarclient)


  INCLUDE_DIRECTORIES(${LOGPOLAR_INCLUDE_DIRS} ${ICUBDEV_INCLUDE_DIRS} 
                      ${PROJECT_SOURCE_DIR}/logpolarGrabber
                      ${YARP_INCLUDE_DIRS})

  ADD_LIBRARY(logpolarclient ClientLogpolarFrameGrabber.cpp)

  TARGET_LINK_LIBRARIES(logpolarclient ${LOGPOLAR_LIBRARIES} 
                                        ${ICUBDEV_LIBRARIES} 
                                        ${YARP_LIBRARIES})
ENDIF(NOT SKIP_logpolarclient)
