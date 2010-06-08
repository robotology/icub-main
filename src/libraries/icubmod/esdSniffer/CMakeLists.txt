PREPARE_DEVICE(esdsniffer TYPE EsdMessageSniffer
    INCLUDE EsdMessageSniffer.h 
    WRAPPER "")

IF(NOT SKIP_esdsniffer)


  FIND_PACKAGE(ESDCANAPI)
  
  IF(NOT ESDCANAPI_FOUND)
    MESSAGE(SEND_ERROR "esdSniffer: cannot find esdcan api, turn off device")
  ELSE(NOT ESDCANAPI_FOUND)
    INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${ESDCANAPI_INC_DIRS} ${YARP_INCLUDE_DIRS})
    ADD_LIBRARY(esdsniffer EsdMessageSniffer.cpp EsdMessageSniffer.h)
    TARGET_LINK_LIBRARIES(esdsniffer  ${ESDCANAPI_LIB} ${YARP_LIBRARIES})
    icub_export_library(esdsniffer)
  ENDIF(NOT ESDCANAPI_FOUND)

ENDIF(NOT SKIP_esdsniffer)
