IF (COMPILE_DEVICE_LIBRARY)
  PREPARE_DEVICE(cartesiancontrollerserver TYPE ServerCartesianController INCLUDE ServerCartesianController.h)
  PREPARE_DEVICE(cartesiancontrollerclient TYPE ClientCartesianController INCLUDE ClientCartesianController.h)
ENDIF (COMPILE_DEVICE_LIBRARY)

IF (NOT SKIP_cartesiancontrollerserver OR NOT SKIP_cartesiancontrollerclient)
  FIND_PACKAGE(YARP)
  FIND_PACKAGE(ICUB)

  IF (NOT SKIP_cartesiancontrollerserver)
    FIND_PACKAGE(IKIN REQUIRED)
    INCLUDE_DIRECTORIES(${IKIN_INCLUDE_DIRS})

    SET(server_source ServerCartesianController.cpp)
    SET(server_header CommonCartesianController.h ServerCartesianController.h)

    ADD_LIBRARY(cartesiancontrollerserver ${server_source} ${server_header})
    LINK_LIBRARIES(${IKIN_LIBRARIES})
  ENDIF (NOT SKIP_cartesiancontrollerserver)

  IF (NOT SKIP_cartesiancontrollerclient)
    SET(client_source ClientCartesianController.cpp)
    SET(client_header CommonCartesianController.h ClientCartesianController.h)
    
    ADD_LIBRARY(cartesiancontrollerclient ${client_source} ${client_header})
  ENDIF (NOT SKIP_cartesiancontrollerclient)

ENDIF (NOT SKIP_cartesiancontrollerserver OR NOT SKIP_cartesiancontrollerclient)

