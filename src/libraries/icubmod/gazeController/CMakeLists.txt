# Copyright: 2010 RobotCub Consortium
# Author: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

yarp_prepare_plugin(gazecontrollerclient CATEGORY device
                                         TYPE ClientGazeController
                                         INCLUDE ClientGazeController.h
                                         EXTRA_CONFIG WRAPPER=gazecontrollerclient)

if(NOT SKIP_gazecontrollerclient)
   set(CMAKE_INCLUDE_CURRENT_DIR ON)
   set(client_source ClientGazeController.cpp)
   set(client_header ClientGazeController.h)
  
   yarp_add_plugin(gazecontrollerclient ${client_source} ${client_header})

   target_link_libraries(gazecontrollerclient ${YARP_LIBRARIES})

   icub_export_plugin(gazecontrollerclient)

  yarp_install(TARGETS gazecontrollerclient
               COMPONENT Runtime
               LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()

