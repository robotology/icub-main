MESSAGE(STATUS "RobotControllerModule Library")

IF (NESTED_BUILD)
  SET(RobotControllerModule_LIBRARIES RobotControllerModule)
ELSE (NESTED_BUILD)
  FIND_LIBRARY(RobotControllerModule_LIBRARIES RobotControllerModule ${RobotControllerModule_DIR}/lib)
ENDIF (NESTED_BUILD)

SET(RobotControllerModule_INCLUDE_DIR ${RobotControllerModule_DIR}/include)

