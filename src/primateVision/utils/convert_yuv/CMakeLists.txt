#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/mmx	
	)

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/mmx	
	)


ADD_LIBRARY(	convert_yuv
		SHARED
		convert_yuv.cc
		)

TARGET_LINK_LIBRARIES(	convert_yuv
                        Qt3Support
                        QtCore
			ippi
			mmx
			)

# install targets
INSTALL_TARGETS(${LIB_DIR} 
  convert_yuv
  )
