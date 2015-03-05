# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_device(logpolargrabber TYPE yarp::dev::ServerLogpolarFrameGrabber
    INCLUDE yarp/dev/ServerLogpolarFrameGrabber.h 
    WRAPPER logpolargrabber)

IF(NOT SKIP_logpolargrabber)
    INCLUDE_DIRECTORIES(${logpolar_INCLUDE_DIRS} ${iCubDev_INCLUDE_DIRS} 
                        ${YARP_INCLUDE_DIRS})
                      
    yarp_add_plugin(logpolargrabber ServerLogpolarFrameGrabber.cpp yarp/dev/ServerLogpolarFrameGrabber.h)
    TARGET_LINK_LIBRARIES(logpolargrabber logpolar 
                                        ${iCubDev_LIBRARIES} 
                                        ${YARP_LIBRARIES})
                                        
    icub_export_plugin(logpolargrabber)
    yarp_install(FILES logpolargrabber.ini  DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})
ENDIF(NOT SKIP_logpolargrabber)
