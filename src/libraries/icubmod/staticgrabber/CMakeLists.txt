# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(static_grabber CATEGORY device
                                   TYPE yarp::dev::StaticGrabber INCLUDE StaticGrabber.h
                                   EXTRA_CONFIG WRAPPER=grabber)

if (NOT SKIP_static_grabber)
  set(CMAKE_INCLUDE_CURRENT_DIR ON)

  yarp_add_plugin(static_grabber StaticGrabber.h StaticGrabber.cpp)

  target_link_libraries(static_grabber ${YARP_LIBRARIES})
  
  icub_export_plugin(static_grabber)
 
  yarp_install(TARGETS static_grabber
               COMPONENT Runtime
               LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

endif (NOT SKIP_static_grabber)


