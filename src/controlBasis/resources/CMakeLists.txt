#/*
# * Copyright (C) 2009 Stephen Hart. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX)

INCLUDE_DIRECTORIES(
  )

LINK_DIRECTORIES(
  )

ADD_LIBRARY(    cbresources
  SHARED
  YARPConfigurationVariables.cpp 
  iCubFullArmConfigurationVariables.cpp 
  EndEffectorCartesianPosition.cpp
  iCubConfigurationReference.cpp
  )

TARGET_LINK_LIBRARIES(  cbresources
  ${IKIN_LIBRARIES}
  )

#install targets
INSTALL_TARGETS(${LIB_DIR}
  cbresources 
  )
