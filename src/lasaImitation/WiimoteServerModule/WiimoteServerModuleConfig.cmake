MESSAGE(STATUS "WiimoteServerModule Library")

#FIND_LIBRARY(WiimoteServerModule_LIBRARIES WiimoteServerModule ${WiimoteServerModule_DIR}/lib)

#IF (NOT WiimoteServerModule_LIBRARIES)
  SET(WiimoteServerModule_LIBRARIES WiimoteServerModule)
#ELSE (NESTED_BUILD)
#  FIND_LIBRARY(WiimoteServerModule_LIBRARIES WiimoteServerModule ${WiimoteServerModule_DIR}/lib)
#ENDIF (NOT WiimoteServerModule_LIBRARIES)

#IF (NESTED_BUILD)
#  SET(WiimoteServerModule_LIBRARIES WiimoteServerModule)
#ELSE (NESTED_BUILD)
#  FIND_LIBRARY(WiimoteServerModule_LIBRARIES WiimoteServerModule ${WiimoteServerModule_DIR}/lib)
#ENDIF (NESTED_BUILD)

SET(WiimoteServerModule_INCLUDE_DIR ${WiimoteServerModule_DIR}/include)

