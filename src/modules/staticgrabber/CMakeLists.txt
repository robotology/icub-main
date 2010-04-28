
PREPARE_DEVICE(static_grabber TYPE StaticGrabber INCLUDE StaticGrabber.h
				WRAPPER grabber)

INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})

ADD_LIBRARY(static_grabber StaticGrabber.h StaticGrabber.cpp)

TARGET_LINK_LIBRARIES(static_grabber ${YARP_LIBRARIES})
