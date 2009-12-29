MESSAGE(STATUS "TouchControllerModule Library")

IF (NESTED_BUILD)
  SET(TouchControllerModule_LIBRARIES TouchControllerModule)
ELSE (NESTED_BUILD)
  FIND_LIBRARY(TouchControllerModule_LIBRARIES TouchControllerModule ${TouchControllerModule_DIR})
ENDIF (NESTED_BUILD)

SET(TouchControllerModule_INCLUDE_DIR ${TouchControllerModule_DIR}/include)

