MESSAGE(STATUS "MultipleMiceExample Library")

IF (NESTED_BUILD)
  SET(MultipleMiceExample_LIBRARIES MultipleMiceExample)
ELSE (NESTED_BUILD)
  FIND_LIBRARY(MultipleMiceExample_LIBRARIES MultipleMiceExample ${MultipleMiceExample_DIR}/lib)
ENDIF (NESTED_BUILD)

SET(MultipleMiceExample_INCLUDE_DIR ${MultipleMiceExample_DIR}/include)

