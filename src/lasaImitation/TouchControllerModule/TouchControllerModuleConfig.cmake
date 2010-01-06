MESSAGE(STATUS "TouchControllerModule Library")

#FIND_LIBRARY(TouchControllerModule_LIBRARIES TouchControllerModule ${TouchControllerModule_DIR}/lib)

#IF (NOT TouchControllerModule_LIBRARIES)
  SET(TouchControllerModule_LIBRARIES TouchControllerModule)
#ELSE (NESTED_BUILD)
#  FIND_LIBRARY(TouchControllerModule_LIBRARIES TouchControllerModule ${TouchControllerModule_DIR}/lib)
#ENDIF (NOT TouchControllerModule_LIBRARIES)

#IF (NESTED_BUILD)
#  SET(TouchControllerModule_LIBRARIES TouchControllerModule)
#ELSE (NESTED_BUILD)
#  FIND_LIBRARY(TouchControllerModule_LIBRARIES TouchControllerModule ${TouchControllerModule_DIR}/lib)
#ENDIF (NESTED_BUILD)

SET(TouchControllerModule_INCLUDE_DIR ${TouchControllerModule_DIR}/include)

