#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

SUBDIRS(ocsclient)

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/recserver 
	${PROJECT_SOURCE_DIR}/utils/centsur
	${PROJECT_SOURCE_DIR}/utils/lgphase
	${PROJECT_SOURCE_DIR}/utils/timing
	${PROJECT_SOURCE_DIR}/utils/display
	${PROJECT_SOURCE_DIR}/utils/multiFrameViewer
	${PROJECT_SOURCE_DIR}/utils/convert_bitdepth
	)

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/centsur
	${PROJECT_SOURCE_DIR}/utils/lgphase
	${PROJECT_SOURCE_DIR}/utils/timing
	${PROJECT_SOURCE_DIR}/utils/display
	${PROJECT_SOURCE_DIR}/utils/convert_bitdepth
	)

ADD_EXECUTABLE( ocsserver
		main.cc 
		ocs.cc
		orient_t.cc
		centsur_t.cc
		)

TARGET_LINK_LIBRARIES(ocsserver
	ippi
	ippcv
	centsur
	lgphase
	timing
        Qt3Support
        QtCore
	display
	convert_bitdepth
	)

# install targets
INSTALL_TARGETS(${BIN_DIR} 
  ocsserver
  )
