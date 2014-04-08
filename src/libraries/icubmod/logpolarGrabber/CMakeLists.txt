# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PREPARE_DEVICE(logpolargrabber TYPE ServerLogpolarFrameGrabber
    INCLUDE yarp/dev/ServerLogpolarFrameGrabber.h 
    WRAPPER logpolargrabber)

IF(NOT SKIP_logpolargrabber)
    INCLUDE_DIRECTORIES(${logpolar_INCLUDE_DIRS} ${iCubDev_INCLUDE_DIRS} 
                        ${YARP_INCLUDE_DIRS})
                      
  yarp_add_plugin(logpolargrabber ServerLogpolarFrameGrabber.cpp yarp/dev/ServerLogpolarFrameGrabber.h)
  TARGET_LINK_LIBRARIES(logpolargrabber logpolar 
                                        ${iCubDev_LIBRARIES} 
                                        ${YARP_LIBRARIES})
                                        
  icub_export_library(logpolargrabber)
                                        
ENDIF(NOT SKIP_logpolargrabber)
