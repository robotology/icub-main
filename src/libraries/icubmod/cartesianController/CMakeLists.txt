# Copyright: 2010 RobotCub Consortium
# Author: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

prepare_device(cartesiancontrollerserver TYPE    ServerCartesianController
                                         INCLUDE ServerCartesianController.h
                                         WRAPPER cartesiancontrollerclient)

if(NOT SKIP_cartesiancontrollerserver)
   set(server_source ServerCartesianController.cpp
                     SmithPredictor.cpp)
   set(server_header CommonCartesianController.h
                     ServerCartesianController.h
                     SmithPredictor.h)

   include_directories(${iKin_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
   yarp_add_plugin(cartesiancontrollerserver ${server_source} ${server_header})
   target_link_libraries(cartesiancontrollerserver iKin ${YARP_LIBRARIES})
   icub_export_library(cartesiancontrollerserver)

endif()

prepare_device(cartesiancontrollerclient TYPE    ClientCartesianController 
                                         INCLUDE ClientCartesianController.h
                                         WRAPPER cartesiancontrollerclient)

if(NOT SKIP_cartesiancontrollerclient)
   set(client_source ClientCartesianController.cpp)
   set(client_header CommonCartesianController.h
                     ClientCartesianController.h)

   include_directories(${iKin_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
   yarp_add_plugin(cartesiancontrollerclient ${client_source} ${client_header})
   target_link_libraries(cartesiancontrollerclient iKin ${YARP_LIBRARIES})

   icub_export_library(cartesiancontrollerclient)
endif()


