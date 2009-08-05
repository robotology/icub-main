#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */


ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

SUBDIRS(zdfclient)

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/multiclass
	${PROJECT_SOURCE_DIR}/utils/timing
	${PROJECT_SOURCE_DIR}/utils/dog
	${PROJECT_SOURCE_DIR}/utils/coord
	${PROJECT_SOURCE_DIR}/utils/canny
	${PROJECT_SOURCE_DIR}/utils/norm
	${PROJECT_SOURCE_DIR}/recserver
	)

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/multiclass
	${PROJECT_SOURCE_DIR}/utils/timing
	${PROJECT_SOURCE_DIR}/utils/dog
	${PROJECT_SOURCE_DIR}/utils/canny
	${PROJECT_SOURCE_DIR}/utils/norm
	)

ADD_EXECUTABLE( zdfserver
		main.cc
		zdf.cc
		)

TARGET_LINK_LIBRARIES(zdfserver
	ippi
	convert_bitdepth
	multiclass
	timing
	canny
	dog
	norm
	Qt3Support
        QtCore
	)

ADD_EXECUTABLE( zdfreset
		zdfreset.cc
		)
ADD_EXECUTABLE( setzero
		setzero.cc
		)


# install targets
INSTALL_TARGETS(${BIN_DIR} 
  zdfserver
  zdfreset
  setzero
  )
