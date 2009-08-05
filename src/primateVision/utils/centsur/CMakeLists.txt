#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

SUBDIRS( testcentsur )

INCLUDE_DIRECTORIES( 
	${PROJECT_SOURCE_DIR}/utils/convert_bitdepth
	)

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/convert_bitdepth
	)

ADD_LIBRARY(	centsur
		SHARED
		centsur.cc
		)

TARGET_LINK_LIBRARIES(	centsur
                        Qt3Support
                        QtCore
			ippi
			convert_bitdepth
			)

# install targets
INSTALL_TARGETS(${LIB_DIR} 
  centsur
  )
