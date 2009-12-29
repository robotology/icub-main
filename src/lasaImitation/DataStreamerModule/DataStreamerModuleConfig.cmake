MESSAGE(STATUS "DataStreamerModule Library")

IF (NESTED_BUILD)
  SET(DataStreamerModule_LIBRARIES DataStreamerModule)
ELSE (NESTED_BUILD)
  FIND_LIBRARY(DataStreamerModule_LIBRARIES DataStreamerModule ${DataStreamerModule_DIR})
ENDIF (NESTED_BUILD)

SET(DataStreamerModule_INCLUDE_DIR ${DataStreamerModule_DIR}/include)

