#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

INCLUDE_DIRECTORIES(
	)

LINK_DIRECTORIES(
	)

ADD_LIBRARY( canny
	     SHARED
	     canny.cpp
	     )

TARGET_LINK_LIBRARIES(canny
        Qt3Support
        QtCore
	ippi
	ippcv
	)


# install targets
INSTALL_TARGETS(${LIB_DIR} 
  canny
  )
