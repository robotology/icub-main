MESSAGE(STATUS "ImitationApplicationModule Library")

#FIND_LIBRARY(ImitationApplicationModule_LIBRARIES ImitationApplicationModule ${ImitationApplicationModule_DIR}/lib)

#IF (NOT ImitationApplicationModule_LIBRARIES)
  SET(ImitationApplicationModule_LIBRARIES ImitationApplicationModule)
#ELSE (NESTED_BUILD)
#  FIND_LIBRARY(ImitationApplicationModule_LIBRARIES ImitationApplicationModule ${ImitationApplicationModule_DIR}/lib)
#ENDIF (NOT ImitationApplicationModule_LIBRARIES)

#IF (NESTED_BUILD)
#  SET(ImitationApplicationModule_LIBRARIES ImitationApplicationModule)
#ELSE (NESTED_BUILD)
#  FIND_LIBRARY(ImitationApplicationModule_LIBRARIES ImitationApplicationModule ${ImitationApplicationModule_DIR}/lib)
#ENDIF (NESTED_BUILD)

SET(ImitationApplicationModule_INCLUDE_DIR ${ImitationApplicationModule_DIR}/include)

