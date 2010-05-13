IF (COMPILE_DEVICE_LIBRARY)
  PREPARE_DEVICE(gazecontrollerclient TYPE ClientGazeController 
                                      INCLUDE ClientGazeController.h WRAPPER gazecontrollerclient)
ENDIF (COMPILE_DEVICE_LIBRARY)

IF (NOT SKIP_gazecontrollerclient)
  FIND_PACKAGE(YARP)
  FIND_PACKAGE(ICUB)
  FIND_PACKAGE(CTRLLIB)

  SET(client_source ClientGazeController.cpp)
  SET(client_header ClientGazeController.h)
  
  INCLUDE_DIRECTORIES(${CTRLLIB_INCLUDE_DIRS})
  ADD_LIBRARY(gazecontrollerclient ${client_source} ${client_header})
  TARGET_LINK_LIBRARIES(gazecontrollerclient ${CTRLLIB_LIBRARIES})

ENDIF (NOT SKIP_gazecontrollerclient)

