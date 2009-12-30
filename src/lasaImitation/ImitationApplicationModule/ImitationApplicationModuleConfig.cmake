MESSAGE(STATUS "ImitationApplicationModule Library")

IF (NESTED_BUILD)
  SET(ImitationApplicationModule_LIBRARIES ImitationApplicationModule)
ELSE (NESTED_BUILD)
  FIND_LIBRARY(ImitationApplicationModule_LIBRARIES ImitationApplicationModule ${ImitationApplicationModule_DIR})
ENDIF (NESTED_BUILD)

SET(ImitationApplicationModule_INCLUDE_DIR ${ImitationApplicationModule_DIR}/include)

