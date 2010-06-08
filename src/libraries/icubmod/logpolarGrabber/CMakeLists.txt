PREPARE_DEVICE(logpolargrabber TYPE ServerLogpolarFrameGrabber
    INCLUDE yarp/dev/ServerLogpolarFrameGrabber.h 
    WRAPPER logpolargrabber)

IF(NOT SKIP_logpolargrabber)
    INCLUDE_DIRECTORIES(${LOGPOLAR_INCLUDE_DIRS} ${ICUBDEV_INCLUDE_DIRS} 
                        ${YARP_INCLUDE_DIRS})
                      
  ADD_LIBRARY(logpolargrabber ServerLogpolarFrameGrabber.cpp yarp/dev/ServerLogpolarFrameGrabber.h)
  
  TARGET_LINK_LIBRARIES(logpolargrabber ${LOGPOLAR_LIBRARIES} 
                                        ${ICUBDEV_LIBRARIES} 
                                        ${YARP_LIBRARIES})
                                        
  icub_export_library(logpolargrabber)
                                        
ENDIF(NOT SKIP_logpolargrabber)
