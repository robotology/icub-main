#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */


ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

INCLUDE_DIRECTORIES(
	${MY_ICUB_DIR}/src/primateVision/utils/timing
	${MY_ICUB_DIR}/src/primateVision/utils/depth
	${MY_ICUB_DIR}/src/primateVision/utils/dog
	${MY_ICUB_DIR}/src/primateVision/utils/coord
	${MY_ICUB_DIR}/src/primateVision/utils/norm
	${MY_ICUB_DIR}/src/primateVision/utils/display
	${MY_ICUB_DIR}/src/primateVision/utils/convert_rgb
	${MY_ICUB_DIR}/src/primateVision/utils/multiFrameViewer
	)

LINK_DIRECTORIES(
	${MY_ICUB_DIR}/src/primateVision/utils/timing
	${MY_ICUB_DIR}/src/primateVision/utils/depth
	${MY_ICUB_DIR}/src/primateVision/utils/dog
	${MY_ICUB_DIR}/src/primateVision/utils/norm
	${MY_ICUB_DIR}/src/primateVision/utils/display
	${MY_ICUB_DIR}/src/primateVision/utils/convert_rgb
	${MY_ICUB_DIR}/src/primateVision/utils/multiFrameViewer
	)
 
ADD_EXECUTABLE( armX
		main.cpp
		armXopt.cpp
		graph.cpp
		maxflow.cpp
		)

TARGET_LINK_LIBRARIES(armX
	display
	multiframeviewer
	convert_rgb
	ippi
	norm
	timing
	depth
	dog
	Qt3Support
        QtCore
        QtGui
        ${OPENCV_LIBRARIES}
	)


# install targets
INSTALL_TARGETS(${BIN_DIR} 
  armX
  )
