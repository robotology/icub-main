MESSAGE(STATUS "VelocityControllerExt Library")

IF (NESTED_BUILD)
  SET(VelocityControllerExt_LIBRARIES VelocityControllerExt)
ELSE (NESTED_BUILD)
  FIND_LIBRARY(VelocityControllerExt_LIBRARIES VelocityControllerExt ${VelocityControllerExt_DIR})
ENDIF (NESTED_BUILD)

SET(VelocityControllerExt_INCLUDE_DIR ${VelocityControllerExt_DIR}/include)

