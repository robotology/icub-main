IF (COMPILE_DEVICE_LIBRARY)
  PREPARE_DEVICE(esdsniffer TYPE EsdMessageSniffer
    INCLUDE EsdMessageSniffer.h 
    WRAPPER "")
ENDIF (COMPILE_DEVICE_LIBRARY)

IF(NOT SKIP_esdsniffer)
  FIND_PACKAGE(YARP REQUIRED)
  IF(WIN32 AND NOT CYGWIN)
    FIND_PATH(inc_esdapi ntcan.h PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../esdCanAPi/winnt/include)
    FIND_LIBRARY(lib_esdapi ntcan ${CMAKE_CURRENT_SOURCE_DIR}/../esdCanAPi/winnt/lib)
  ELSE(WIN32 AND NOT CYGWIN)
    FIND_PATH(inc_esdapi ntcan.h PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../esdCanAPi/linux/include)
    FIND_LIBRARY(lib_esdapi ntcan ${CMAKE_CURRENT_SOURCE_DIR}/../esdCanAPi/linux/lib)
  ENDIF(WIN32 AND NOT CYGWIN)
  
  IF(inc_esdapi)
	IF (lib_esdapi)
      INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/common)
	  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../motionControlLib/)
      INCLUDE_DIRECTORIES(${inc_esdapi})
      LINK_LIBRARIES(${lib_esdapi})

      ADD_LIBRARY(esdsniffer EsdMessageSniffer.cpp EsdMessageSniffer.h)

	ELSE (lib_esdapi)
      MESSAGE(SEND_ERROR "Cannot find ntcan library")
	ENDIF (lib_esdapi)

  ELSE(inc_esdapi)
	MESSAGE(SEND_ERROR "Cannot find ntcan.h")
  ENDIF(inc_esdapi)
ENDIF(NOT SKIP_esdsniffer)
