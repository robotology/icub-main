#/*
# * Copyright (C) 2009 Stephen Hart. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX)

INCLUDE_DIRECTORIES(
  )

LINK_DIRECTORIES(
  )

ADD_LIBRARY(    cbjacobian
#  SHARED
  ManipulatorPositionJacobian.cpp
  )

TARGET_LINK_LIBRARIES(  cbjacobian
  )

#install targets
INSTALL_TARGETS(${LIB_DIR}
  cbjacobian
  )
