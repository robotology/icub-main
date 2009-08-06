
FIND_PACKAGE(YARP REQUIRED)

IF (COMPILE_DEVICE_LIBRARY)
  PREPARE_DEVICE(static_grabber TYPE StaticGrabber INCLUDE StaticGrabber.h
				WRAPPER grabber)
ENDIF (COMPILE_DEVICE_LIBRARY)

ADD_LIBRARY(static_grabber StaticGrabber.h StaticGrabber.cpp)
