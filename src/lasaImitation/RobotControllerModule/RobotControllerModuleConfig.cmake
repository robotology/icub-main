MESSAGE(STATUS "RobotControllerModule Library")

#FIND_LIBRARY(RobotControllerModule_LIBRARIES RobotControllerModule ${RobotControllerModule_DIR}/lib)

#IF (NOT RobotControllerModule_LIBRARIES)
  SET(RobotControllerModule_LIBRARIES RobotControllerModule)
#ELSE (NESTED_BUILD)
#  FIND_LIBRARY(RobotControllerModule_LIBRARIES RobotControllerModule ${RobotControllerModule_DIR}/lib)
#ENDIF (NOT RobotControllerModule_LIBRARIES)

#IF (NESTED_BUILD)
#  SET(RobotControllerModule_LIBRARIES RobotControllerModule)
#ELSE (NESTED_BUILD)
#  FIND_LIBRARY(RobotControllerModule_LIBRARIES RobotControllerModule ${RobotControllerModule_DIR}/lib)
#ENDIF (NESTED_BUILD)

SET(RobotControllerModule_INCLUDE_DIR ${RobotControllerModule_DIR}/include)

