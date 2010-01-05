MESSAGE(STATUS "YarpedMotionSensors Library")

IF (NESTED_BUILD)
  SET(YarpedMotionSensors_LIBRARIES YarpedMotionSensors)
ELSE (NESTED_BUILD)
  FIND_LIBRARY(YarpedMotionSensors_LIBRARIES YarpedMotionSensors ${YarpedMotionSensors_DIR}/lib)
ENDIF (NESTED_BUILD)

SET(YarpedMotionSensors_INCLUDE_DIR ${YarpedMotionSensors_DIR}/include)

