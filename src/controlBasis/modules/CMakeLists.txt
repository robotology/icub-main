#/*
# * Copyright (C) 2010 Stephen Hart. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX)

INCLUDE_DIRECTORIES(
  ${PROJECT_SOURCE_DIR}/resources
  )

LINK_DIRECTORIES(
  ${PROJECT_SOURCE_DIR}/resources
  )

ADD_EXECUTABLE( controlBasisResource
  main.cpp
  )

TARGET_LINK_LIBRARIES(  controlBasisResource
  cbresources
  ${IKIN_LIBRARIES}
  )

INSTALL_TARGETS(../../../bin
  controlBasisResource
  )

