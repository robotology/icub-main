#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */



ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)


SUBDIRS(recclient)


INCLUDE_DIRECTORIES(
	/usr/include/meschach
	${PROJECT_SOURCE_DIR}/utils/convert_rgb
	${PROJECT_SOURCE_DIR}/utils/rectify
	${PROJECT_SOURCE_DIR}/utils/timing
	)

LINK_DIRECTORIES(
	/usr/lib/meschach
	${PROJECT_SOURCE_DIR}/utils/convert_rgb
	${PROJECT_SOURCE_DIR}/utils/rectify
	${PROJECT_SOURCE_DIR}/utils/timing
	)

ADD_EXECUTABLE( recserver
		main.cc 
		rec.cc
		recmot.cc
		)

TARGET_LINK_LIBRARIES(recserver
	rectify
	meschach
	ippi
	convert_rgb
        Qt3Support
        QtCore
	timing
	)

# install targets
INSTALL_TARGETS(${BIN_DIR} 
  recserver
  )
