 #/*
 #* Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 #* 
 #*/

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

SUBDIRS(testdepth)

INCLUDE_DIRECTORIES(
        )

LINK_DIRECTORIES(
	) 


ADD_LIBRARY( depth
	     depth.cc
	     )

TARGET_LINK_LIBRARIES(depth
        Qt3Support
        QtCore
	ippi
        ${OPENCV_LIBRARIES}
	)

# install targets
INSTALL_TARGETS(${LIB_DIR} 
  depth
  )
