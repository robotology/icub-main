

IF (COMPILE_DEVICE_LIBRARY)
  PREPARE_DEVICE(static_grabber TYPE StaticGrabber INCLUDE StaticGrabber.h
				WRAPPER grabber)
ENDIF (COMPILE_DEVICE_LIBRARY)

IF (NOT SKIP_static_grabber)
FIND_PACKAGE(YARP REQUIRED)

ADD_LIBRARY(static_grabber StaticGrabber.h StaticGrabber.cpp)
ENDIF (NOT SKIP_static_grabber)