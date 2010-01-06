MESSAGE(STATUS "DataStreamerModule Library")

#FIND_LIBRARY(DataStreamerModule_LIBRARIES DataStreamerModule ${DataStreamerModule_DIR}/lib)

#IF (NOT DataStreamerModule_LIBRARIES)
  SET(DataStreamerModule_LIBRARIES DataStreamerModule)
#ELSE (NESTED_BUILD)
#  FIND_LIBRARY(DataStreamerModule_LIBRARIES DataStreamerModule ${DataStreamerModule_DIR}/lib)
#ENDIF (NOT DataStreamerModule_LIBRARIES)

#IF (NESTED_BUILD)
#  SET(DataStreamerModule_LIBRARIES DataStreamerModule)
#ELSE (NESTED_BUILD)
#  FIND_LIBRARY(DataStreamerModule_LIBRARIES DataStreamerModule ${DataStreamerModule_DIR}/lib)
#ENDIF (NESTED_BUILD)

SET(DataStreamerModule_INCLUDE_DIR ${DataStreamerModule_DIR}/include)

