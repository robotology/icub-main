#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

ADD_LIBRARY(	lgphase
		SHARED
		lgphase.cc
		)

TARGET_LINK_LIBRARIES(	lgphase
                        Qt3Support
                        QtCore
			ippi
			)

# install targets
INSTALL_TARGETS(${LIB_DIR} 
  lgphase
  )
