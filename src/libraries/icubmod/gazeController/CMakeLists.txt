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
  
   include_directories(${YARP_INCLUDE_DIRS})

   yarp_add_plugin(gazecontrollerclient ${client_source} ${client_header})

   target_link_libraries(gazecontrollerclient ${YARP_LIBRARIES})

   icub_export_plugin(gazecontrollerclient)
   yarp_install(FILES gazecontrollerclient.ini DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})
endif()

