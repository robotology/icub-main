# Copyright: 2010 RobotCub Consortium
# Author: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

yarp_prepare_device(gazecontrollerclient TYPE ClientGazeController
                                    INCLUDE ClientGazeController.h
                                    WRAPPER gazecontrollerclient)

IF (NOT SKIP_gazecontrollerclient)
   set(CMAKE_INCLUDE_CURRENT_DIR ON)
   SET(client_source ClientGazeController.cpp)
   SET(client_header ClientGazeController.h)
  
   INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})

   yarp_add_plugin(gazecontrollerclient ${client_source} ${client_header})

   TARGET_LINK_LIBRARIES(gazecontrollerclient ${YARP_LIBRARIES})

   icub_export_plugin(gazecontrollerclient)
   yarp_install(FILES gazecontrollerclient.ini  DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})
ENDIF (NOT SKIP_gazecontrollerclient)

