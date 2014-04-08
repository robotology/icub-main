# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PREPARE_DEVICE(logpolarclient TYPE ClientLogpolarFrameGrabber
    INCLUDE yarp/dev/ClientLogpolarFrameGrabber.h)

IF(NOT SKIP_logpolarclient)


  INCLUDE_DIRECTORIES(${iCubDev_INCLUDE_DIRS} 
	                  ${logpolar_INCLUDE_DIRS}
                      ${PROJECT_SOURCE_DIR}/logpolarGrabber
                      ${YARP_INCLUDE_DIRS})

  yarp_add_plugin(logpolarclient ClientLogpolarFrameGrabber.cpp)

  TARGET_LINK_LIBRARIES(logpolarclient  iCubDev 
										logpolar
                                        ${YARP_LIBRARIES})
                                        
   icub_export_library(logpolarclient)
ENDIF(NOT SKIP_logpolarclient)
