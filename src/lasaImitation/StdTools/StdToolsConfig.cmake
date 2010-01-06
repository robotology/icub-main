MESSAGE(STATUS "StdTools Library")

#FIND_LIBRARY(StdTools_LIBRARIES StdTools ${StdTools_DIR}/lib)

#IF (NOT StdTools_LIBRARIES)
  SET(StdTools_LIBRARIES StdTools)
#ELSE (NESTED_BUILD)
#  FIND_LIBRARY(StdTools_LIBRARIES StdTools ${StdTools_DIR}/lib)
#ENDIF (NOT StdTools_LIBRARIES)

#IF (NESTED_BUILD)
#  SET(StdTools_LIBRARIES StdTools)
#ELSE (NESTED_BUILD)
#  FIND_LIBRARY(StdTools_LIBRARIES StdTools ${StdTools_DIR}/lib)
#ENDIF (NESTED_BUILD)

SET(StdTools_INCLUDE_DIR ${StdTools_DIR}/include)

