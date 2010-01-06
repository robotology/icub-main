MESSAGE(STATUS "UDPConnection Library")

#FIND_LIBRARY(UDPConnection_LIBRARIES UDPConnection ${UDPConnection_DIR}/lib)

#IF (NOT UDPConnection_LIBRARIES)
  SET(UDPConnection_LIBRARIES UDPConnection)
#ELSE (NESTED_BUILD)
#  FIND_LIBRARY(UDPConnection_LIBRARIES UDPConnection ${UDPConnection_DIR}/lib)
#ENDIF (NOT UDPConnection_LIBRARIES)

#IF (NESTED_BUILD)
#  SET(UDPConnection_LIBRARIES UDPConnection)
#ELSE (NESTED_BUILD)
#  FIND_LIBRARY(UDPConnection_LIBRARIES UDPConnection ${UDPConnection_DIR}/lib)
#ENDIF (NESTED_BUILD)

SET(UDPConnection_INCLUDE_DIR ${UDPConnection_DIR}/include)

