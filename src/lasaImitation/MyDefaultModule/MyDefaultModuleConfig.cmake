MESSAGE(STATUS "MyDefaultModule Library")

IF (NESTED_BUILD)
  SET(MyDefaultModule_LIBRARIES MyDefaultModule)
ELSE (NESTED_BUILD)
  FIND_LIBRARY(MyDefaultModule_LIBRARIES MyDefaultModule ${MyDefaultModule_DIR})
ENDIF (NESTED_BUILD)

SET(MyDefaultModule_INCLUDE_DIR ${MyDefaultModule_DIR}/include)

