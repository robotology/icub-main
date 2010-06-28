PREPARE_DEVICE(logpolargrabber TYPE ServerLogpolarFrameGrabber
    INCLUDE yarp/dev/ServerLogpolarFrameGrabber.h 
    WRAPPER logpolargrabber)

IF(NOT SKIP_logpolargrabber)
    INCLUDE_DIRECTORIES(${logPolar_INCLUDE_DIRS} ${iCubDev_INCLUDE_DIRS} 
                        ${YARP_INCLUDE_DIRS})
                      
  ADD_LIBRARY(logpolargrabber ServerLogpolarFrameGrabber.cpp yarp/dev/ServerLogpolarFrameGrabber.h)
  
  TARGET_LINK_LIBRARIES(logpolargrabber ${logPolar_LIBRARIES} 
                                        ${iCubDev_LIBRARIES} 
                                        ${YARP_LIBRARIES})
                                        
  icub_export_library(logpolargrabber)
                                        
ENDIF(NOT SKIP_logpolargrabber)
