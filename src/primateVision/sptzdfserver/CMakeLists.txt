#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */


ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

SUBDIRS(sptzdfclient)

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/multiclass
	${PROJECT_SOURCE_DIR}/utils/timing
	${PROJECT_SOURCE_DIR}/utils/dog
	${PROJECT_SOURCE_DIR}/utils/coord
	${PROJECT_SOURCE_DIR}/utils/norm
	${PROJECT_SOURCE_DIR}/utils/convert_bitdepth
	${PROJECT_SOURCE_DIR}/recserver
	)

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/multiclass
	${PROJECT_SOURCE_DIR}/utils/timing
	${PROJECT_SOURCE_DIR}/utils/dog
	${PROJECT_SOURCE_DIR}/utils/norm
	${PROJECT_SOURCE_DIR}/utils/convert_bitdepth
	)

ADD_EXECUTABLE( sptzdfserver
		main.cc
		sptzdf.cc
		)

TARGET_LINK_LIBRARIES(sptzdfserver
	ippi
	ippcc
	ippcv
	norm
	convert_bitdepth
	multiclass
	timing
	Qt3Support
        QtCore
	dog
	)



# install targets
INSTALL_TARGETS(${BIN_DIR} 
  sptzdfserver
  )
