PREPARE_DEVICE(gazecontrollerclient TYPE    ClientGazeController 
                                    INCLUDE ClientGazeController.h
                                    WRAPPER gazecontrollerclient)

IF (NOT SKIP_gazecontrollerclient)
   SET(client_source ClientGazeController.cpp)
   SET(client_header ClientGazeController.h)
  
   INCLUDE_DIRECTORIES(${ctrlLib_INCLUDE_DIRS}
                       ${YARP_INCLUDE_DIRS})

   ADD_LIBRARY(gazecontrollerclient ${client_source} ${client_header})

   TARGET_LINK_LIBRARIES(gazecontrollerclient ctrlLib
                                              ${YARP_LIBRARIES})

   icub_export_library(gazecontrollerclient)
ENDIF (NOT SKIP_gazecontrollerclient)

