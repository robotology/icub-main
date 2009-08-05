#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)


SUBDIRS(ycsclient)


INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/recserver 
	${PROJECT_SOURCE_DIR}/utils/timing
	${PROJECT_SOURCE_DIR}/utils/centsur
	${PROJECT_SOURCE_DIR}/utils/convert_bitdepth
	)

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/timing
	${PROJECT_SOURCE_DIR}/utils/centsur
	${PROJECT_SOURCE_DIR}/utils/convert_bitdepth
	)

ADD_EXECUTABLE( ycsserver
		main.cc 
		ycs.cc
		)

TARGET_LINK_LIBRARIES(ycsserver
	ippi
	ippcv
	centsur
        Qt3Support
        QtCore	timing
	convert_bitdepth
	)


# install targets
INSTALL_TARGETS(${BIN_DIR} 
  ycsserver
  )
