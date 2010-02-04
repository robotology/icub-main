#/*
# * Copyright (C) 2009 Stephen Hart. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX)

INCLUDE_DIRECTORIES(
  )

LINK_DIRECTORIES(
  )

ADD_LIBRARY(    cbpf
  SHARED
  ConfigurationSquaredError.cpp
  CartesianPositionSquaredError.cpp
  CartesianPositionHarmonicFunction.cpp
  CosineField.cpp
  ManipulabilityField.cpp
  )

TARGET_LINK_LIBRARIES(  cbpf
  )

#install targets
INSTALL_TARGETS(${LIB_DIR}
  cbpf
  )
