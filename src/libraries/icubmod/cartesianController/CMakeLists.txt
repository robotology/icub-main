# Copyright: 2010 RobotCub Consortium
# Author: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

yarp_prepare_plugin(cartesiancontrollerserver CATEGORY device
                                              TYPE ServerCartesianController
                                              INCLUDE ServerCartesianController.h
                                              EXTRA_CONFIG WRAPPER=cartesiancontrollerclient)

if(NOT SKIP_cartesiancontrollerserver)
   set(CMAKE_INCLUDE_CURRENT_DIR ON)
   set(server_source ServerCartesianController.cpp
                     SmithPredictor.cpp)
   set(server_header CommonCartesianController.h
                     ServerCartesianController.h
                     SmithPredictor.h)

   yarp_add_plugin(cartesiancontrollerserver ${server_source} ${server_header})
   target_link_libraries(cartesiancontrollerserver iKin ${YARP_LIBRARIES})
   icub_export_plugin(cartesiancontrollerserver)
      yarp_install(TARGETS cartesiancontrollerserver
               COMPONENT Runtime
               LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})
endif()

yarp_prepare_plugin(cartesiancontrollerclient CATEGORY device
                                              TYPE ClientCartesianController
                                              INCLUDE ClientCartesianController.h
                                              EXTRA_CONFIG WRAPPER=cartesiancontrollerclient)

if(NOT SKIP_cartesiancontrollerclient)
   set(CMAKE_INCLUDE_CURRENT_DIR ON)
   set(client_source ClientCartesianController.cpp)
   set(client_header CommonCartesianController.h
                     ClientCartesianController.h)

   yarp_add_plugin(cartesiancontrollerclient ${client_source} ${client_header})
   target_link_libraries(cartesiancontrollerclient iKin ${YARP_LIBRARIES})

   icub_export_plugin(cartesiancontrollerclient)
     yarp_install(TARGETS cartesiancontrollerclient
               COMPONENT Runtime
               LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})
endif()



