#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# *
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

INCLUDE_DIRECTORIES(
	/usr/include/meschach
	)

LINK_DIRECTORIES(
	/usr/lib/meschach
	)

ADD_LIBRARY( rectify
	     SHARED
	     rectify.cc
	    )

TARGET_LINK_LIBRARIES(rectify
	ippi
	ippcv
	meschach
	)

# install targets
INSTALL_TARGETS(${LIB_DIR} 
  rectify
  )
