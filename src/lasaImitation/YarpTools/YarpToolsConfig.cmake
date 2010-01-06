MESSAGE(STATUS "YarpTools Library")

#FIND_LIBRARY(YarpTools_LIBRARIES YarpTools ${YarpTools_DIR}/lib)

#IF (NOT YarpTools_LIBRARIES)
  SET(YarpTools_LIBRARIES YarpTools)
#ELSE (NESTED_BUILD)
#  FIND_LIBRARY(YarpTools_LIBRARIES YarpTools ${YarpTools_DIR}/lib)
#ENDIF (NOT YarpTools_LIBRARIES)

#IF (NESTED_BUILD)
#  SET(YarpTools_LIBRARIES YarpTools)
#ELSE (NESTED_BUILD)
#  FIND_LIBRARY(YarpTools_LIBRARIES YarpTools ${YarpTools_DIR}/lib)
#ENDIF (NESTED_BUILD)

SET(YarpTools_INCLUDE_DIR ${YarpTools_DIR}/include)

