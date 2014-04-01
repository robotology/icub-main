# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PREPARE_DEVICE(ecan TYPE EsdCan
    INCLUDE EsdCan.h 
    WRAPPER controlboard)

IF (NOT SKIP_ecan)
  FIND_PACKAGE(ESDCANAPI)
  
  IF(NOT ESDCANAPI_FOUND)
    MESSAGE(SEND_ERROR "ecan: cannot find esdcan api, turn off device")
  ELSE(NOT ESDCANAPI_FOUND)
    # Print out library location
    MESSAGE(STATUS " ++++ ESD CAN library found at: ${ESDCANAPI_LIB}")
    MESSAGE(STATUS " ++++ ESD CAN include files found at: ${ESDCANAPI_INC_DIRS}")
    
    INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${ESDCANAPI_INC_DIRS} ${YARP_INCLUDE_DIRS})
    yarp_add_plugin(esdCan EsdCan.cpp EsdCan.h)
    
    TARGET_LINK_LIBRARIES(esdCan ${ESDCANAPI_LIB} ${YARP_LIBRARIES})
    
    icub_export_library(esdCan)
  ENDIF(NOT ESDCANAPI_FOUND)

ENDIF (NOT SKIP_ecan)
