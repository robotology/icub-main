MESSAGE(STATUS "UDPConnection Library")

IF (NESTED_BUILD)
  SET(UDPConnection_LIBRARIES UDPConnection)
ELSE (NESTED_BUILD)
  FIND_LIBRARY(UDPConnection_LIBRARIES UDPConnection ${UDPConnection_DIR}/lib)
ENDIF (NESTED_BUILD)

SET(UDPConnection_INCLUDE_DIR ${UDPConnection_DIR}/include)

