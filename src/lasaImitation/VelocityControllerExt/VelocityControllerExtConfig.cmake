MESSAGE(STATUS "VelocityControllerExt Library")

#FIND_LIBRARY(VelocityControllerExt_LIBRARIES VelocityControllerExt ${VelocityControllerExt_DIR}/lib)

#IF (NOT VelocityControllerExt_LIBRARIES)
  SET(VelocityControllerExt_LIBRARIES VelocityControllerExt)
#ELSE (NESTED_BUILD)
#  FIND_LIBRARY(VelocityControllerExt_LIBRARIES VelocityControllerExt ${VelocityControllerExt_DIR}/lib)
#ENDIF (NOT VelocityControllerExt_LIBRARIES)

#IF (NESTED_BUILD)
#  SET(VelocityControllerExt_LIBRARIES VelocityControllerExt)
#ELSE (NESTED_BUILD)
#  FIND_LIBRARY(VelocityControllerExt_LIBRARIES VelocityControllerExt ${VelocityControllerExt_DIR}/lib)
#ENDIF (NESTED_BUILD)

SET(VelocityControllerExt_INCLUDE_DIR ${VelocityControllerExt_DIR}/include)

