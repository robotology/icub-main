MESSAGE(STATUS "MultipleMiceDriver Library")

#FIND_LIBRARY(MultipleMiceDriver_LIBRARIES MultipleMiceDriver ${MultipleMiceDriver_DIR}/lib)

#IF (NOT MultipleMiceDriver_LIBRARIES)
  SET(MultipleMiceDriver_LIBRARIES MultipleMiceDriver)
#ELSE (NESTED_BUILD)
#  FIND_LIBRARY(MultipleMiceDriver_LIBRARIES MultipleMiceDriver ${MultipleMiceDriver_DIR}/lib)
#ENDIF (NOT MultipleMiceDriver_LIBRARIES)

#IF (NESTED_BUILD)
#  SET(MultipleMiceDriver_LIBRARIES MultipleMiceDriver)
#ELSE (NESTED_BUILD)
#  FIND_LIBRARY(MultipleMiceDriver_LIBRARIES MultipleMiceDriver ${MultipleMiceDriver_DIR}/lib)
#ENDIF (NESTED_BUILD)

SET(MultipleMiceDriver_INCLUDE_DIR ${MultipleMiceDriver_DIR}/include)

