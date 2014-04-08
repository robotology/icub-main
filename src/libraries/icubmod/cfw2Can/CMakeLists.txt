# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PREPARE_DEVICE(cfw2can TYPE Cfw2Can
    INCLUDE Cfw2Can.h 
    WRAPPER "")

IF (NOT SKIP_cfw2can)
  FIND_PACKAGE(CFW2CANAPI)

  IF (NOT CFW2CANAPI_FOUND)
    MESSAGE("cfw2can: cannot find cfw2canapi, turn off device")
  ELSE (NOT CFW2CANAPI_FOUND)
  
    INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${CFW2CANAPI_INC_DIRS} ${YARP_INCLUDE_DIRS})

    yarp_add_plugin(cfw2can Cfw2Can.cpp Cfw2Can.h)
    TARGET_LINK_LIBRARIES(cfw2can ${CFW2CANAPI_LIB} pthread ${YARP_LIBRARIES})
    icub_export_library(cfw2can)
  ENDIF (NOT CFW2CANAPI_FOUND)
ENDIF (NOT SKIP_cfw2can)

