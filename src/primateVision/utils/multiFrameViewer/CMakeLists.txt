#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

ADD_CUSTOM_COMMAND(  OUTPUT mmultiFrameViewer.cc
                     COMMAND moc-qt4 multiFrameViewer.h -o mmultiFrameViewer.cc
                     DEPENDS multiFrameViewer.h multiFrameViewer.cc)


ADD_EXECUTABLE( mfexample 
		mfexample.cc
		multiFrameViewer.cc 
		mmultiFrameViewer.cc 
		)

TARGET_LINK_LIBRARIES( mfexample 
                        Qt3Support 
                        QtCore
                        QtGui
			timing
			)

ADD_LIBRARY( multiframeviewer
             SHARED
	     multiFrameViewer.cc
	     mmultiFrameViewer.cc 
	    )
INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/timing
	)
LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/timing
	)
TARGET_LINK_LIBRARIES( multiframeviewer
  timing
)


# install targets
INSTALL_TARGETS(${LIB_DIR} 
  multiframeviewer
  )

# install targets
INSTALL_TARGETS(${BIN_DIR} 
  mfexample
  )
