#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

ADD_LIBRARY(	convert_bitdepth
		SHARED
		convert_bitdepth.cc
		)

TARGET_LINK_LIBRARIES(	convert_bitdepth
                        Qt3Support
                        QtCore
			ippi
			ippcc
			)

# install targets
INSTALL_TARGETS(${LIB_DIR} 
  convert_bitdepth
  ) 
