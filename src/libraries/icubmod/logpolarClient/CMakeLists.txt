PREPARE_DEVICE(logpolarclient TYPE ClientLogpolarFrameGrabber
    INCLUDE yarp/dev/ClientLogpolarFrameGrabber.h)

IF(NOT SKIP_logpolarclient)


  INCLUDE_DIRECTORIES(${iCubDev_INCLUDE_DIRS} 
	                  ${logPolar_INCLUDE_DIRS}
                      ${PROJECT_SOURCE_DIR}/logpolarGrabber
                      ${YARP_INCLUDE_DIRS})

  ADD_LIBRARY(logpolarclient ClientLogpolarFrameGrabber.cpp)

  MESSAGE("--> ${logPolar_LIBRARIES}")
  MESSAGE("--> ${iCubDev_LIBRARIES}")
  MESSAGE("--> ${YARP_LIBRARIES}")
  TARGET_LINK_LIBRARIES(logpolarclient  iCubDev 
										logPolar
                                        ${YARP_LIBRARIES})
                                        
   icub_export_library(logpolarclient)
ENDIF(NOT SKIP_logpolarclient)
