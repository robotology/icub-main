PREPARE_DEVICE(static_grabber TYPE StaticGrabber INCLUDE StaticGrabber.h
				WRAPPER grabber)

if (NOT SKIP_static_grabber)
  include_directories(${YARP_INCLUDE_DIRS})

  add_library(static_grabber StaticGrabber.h StaticGrabber.cpp)

  target_link_libraries(static_grabber ${YARP_LIBRARIES})
endif (NOT SKIP_static_grabber)


