#/*
# * Copyright (C) 2009 Stephen Hart. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX)

INCLUDE_DIRECTORIES(
  ${PROJECT_SOURCE_DIR}/resources
  ${PROJECT_SOURCE_DIR}/potentialFunctions
  ${PROJECT_SOURCE_DIR}/jacobians
  )

LINK_DIRECTORIES(
  ${PROJECT_SOURCE_DIR}/resources
  ${PROJECT_SOURCE_DIR}/potentialFunctions
  ${PROJECT_SOURCE_DIR}/jacobians
  )

ADD_LIBRARY(    cbcontrol
  SHARED
  Controller.cpp
  RunnableControlLaw.cpp
  )

TARGET_LINK_LIBRARIES(  cbcontrol
  cbresources
  cbpf
  cbjacobian
  )

#install targets
INSTALL_TARGETS(${LIB_DIR}
cbcontrol
  )
