PREPARE_DEVICE(cartesiancontrollerserver TYPE ServerCartesianController
                                           INCLUDE ServerCartesianController.h WRAPPER cartesiancontrollerclient)
PREPARE_DEVICE(cartesiancontrollerclient TYPE ClientCartesianController 
                                           INCLUDE ClientCartesianController.h WRAPPER cartesiancontrollerclient)

IF (NOT SKIP_cartesiancontrollerserver OR NOT SKIP_cartesiancontrollerclient)

  IF (NOT SKIP_cartesiancontrollerserver)
    SET(server_source ServerCartesianController.cpp)
    SET(server_header CommonCartesianController.h ServerCartesianController.h)

    INCLUDE_DIRECTORIES(${ACE_INCLUDE_DIRS})
    INCLUDE_DIRECTORIES(${iKin_INCLUDE_DIRS})
    INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})
    ADD_LIBRARY(cartesiancontrollerserver ${server_source} ${server_header})
    TARGET_LINK_LIBRARIES(cartesiancontrollerserver ${ACE_LIBRARIES} iKin ${YARP_LIBRARIES})
  
    icub_export_library(cartesiancontrollerserver "" include/iCub "")
  ENDIF (NOT SKIP_cartesiancontrollerserver)

  IF (NOT SKIP_cartesiancontrollerclient)
    SET(client_source ClientCartesianController.cpp)
    SET(client_header CommonCartesianController.h ClientCartesianController.h)
    
    INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})

    ADD_LIBRARY(cartesiancontrollerclient ${client_source} ${client_header})
    TARGET_LINK_LIBRARIES(cartesiancontrollerclient ${YARP_LIBRARIES})

    icub_export_library(cartesiancontrollerclient "" include/iCub "")

    
  ENDIF (NOT SKIP_cartesiancontrollerclient)

ENDIF (NOT SKIP_cartesiancontrollerserver OR NOT SKIP_cartesiancontrollerclient)

