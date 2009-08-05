#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/display 
	${PROJECT_SOURCE_DIR}/utils/mosaic
	${PROJECT_SOURCE_DIR}/utils/multiFrameViewer
	${PROJECT_SOURCE_DIR}/recserver
	)

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/display 
	${PROJECT_SOURCE_DIR}/utils/mosaic
	)

ADD_EXECUTABLE( recclient
		main.cc
		)

TARGET_LINK_LIBRARIES(recclient
        Qt3Support
        QtCore
        QtGui
	display
	mosaic
	pthread
	
)
ADD_EXECUTABLE( recclient_motion
		main_mot.cc
		)

TARGET_LINK_LIBRARIES(recclient_motion
        Qt3Support
        QtCore
	display
	mosaic
	pthread
	)

# install targets
INSTALL_TARGETS(${BIN_DIR} 
  recclient
  recclient_motion
  )
