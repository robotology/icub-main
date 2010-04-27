
PREPARE_PLUGIN(static_grabber TYPE StaticGrabber INCLUDE StaticGrabber.h
				WRAPPER grabber)

ADD_LIBRARY(static_grabber StaticGrabber.h StaticGrabber.cpp)
