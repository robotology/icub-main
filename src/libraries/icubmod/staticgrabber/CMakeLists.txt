# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PREPARE_DEVICE(static_grabber TYPE StaticGrabber INCLUDE StaticGrabber.h
				WRAPPER grabber)

if (NOT SKIP_static_grabber)
  include_directories(${YARP_INCLUDE_DIRS})

  yarp_add_plugin(static_grabber StaticGrabber.h StaticGrabber.cpp)

  target_link_libraries(static_grabber ${YARP_LIBRARIES})
  
  icub_export_library(static_grabber)

endif (NOT SKIP_static_grabber)


