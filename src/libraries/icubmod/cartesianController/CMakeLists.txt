# Copyright: 2010 RobotCub Consortium
# Author: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

yarp_prepare_device(cartesiancontrollerserver TYPE ServerCartesianController
                                         INCLUDE ServerCartesianController.h
                                         WRAPPER cartesiancontrollerclient)

if(NOT SKIP_cartesiancontrollerserver)
   set(CMAKE_INCLUDE_CURRENT_DIR ON)
   set(server_source ServerCartesianController.cpp
                     SmithPredictor.cpp)
   set(server_header CommonCartesianController.h
                     ServerCartesianController.h
                     SmithPredictor.h)

   include_directories(${iKin_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
   yarp_add_plugin(cartesiancontrollerserver ${server_source} ${server_header})
   target_link_libraries(cartesiancontrollerserver iKin ${YARP_LIBRARIES})
   icub_export_plugin(cartesiancontrollerserver)
   yarp_install(FILES cartesiancontrollerserver.ini  DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})
endif()

yarp_prepare_device(cartesiancontrollerclient TYPE ClientCartesianController
                                         INCLUDE ClientCartesianController.h
                                         WRAPPER cartesiancontrollerclient)

if(NOT SKIP_cartesiancontrollerclient)
   set(CMAKE_INCLUDE_CURRENT_DIR ON)
   set(client_source ClientCartesianController.cpp)
   set(client_header CommonCartesianController.h
                     ClientCartesianController.h)

   include_directories(${iKin_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
   yarp_add_plugin(cartesiancontrollerclient ${client_source} ${client_header})
   target_link_libraries(cartesiancontrollerclient iKin ${YARP_LIBRARIES})

   icub_export_plugin(cartesiancontrollerclient)
   yarp_install(FILES cartesiancontrollerclient.ini  DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})
endif()



