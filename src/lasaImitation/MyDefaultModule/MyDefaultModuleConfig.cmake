MESSAGE(STATUS "MyDefaultModule Library")

#FIND_LIBRARY(MyDefaultModule_LIBRARIES MyDefaultModule ${MyDefaultModule_DIR}/lib)

#IF (NOT MyDefaultModule_LIBRARIES)
  SET(MyDefaultModule_LIBRARIES MyDefaultModule)
#ELSE (NESTED_BUILD)
#  FIND_LIBRARY(MyDefaultModule_LIBRARIES MyDefaultModule ${MyDefaultModule_DIR}/lib)
#ENDIF (NOT MyDefaultModule_LIBRARIES)

#IF (NESTED_BUILD)
#  SET(MyDefaultModule_LIBRARIES MyDefaultModule)
#ELSE (NESTED_BUILD)
#  FIND_LIBRARY(MyDefaultModule_LIBRARIES MyDefaultModule ${MyDefaultModule_DIR}/lib)
#ENDIF (NESTED_BUILD)

SET(MyDefaultModule_INCLUDE_DIR ${MyDefaultModule_DIR}/include)

