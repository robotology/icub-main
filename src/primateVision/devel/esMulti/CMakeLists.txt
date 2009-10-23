#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

ADD_CUSTOM_COMMAND(  OUTPUT ${PROJECT_SOURCE_DIR}/devel/esMulti/mworld3d.cc
                     COMMAND moc-qt4 ${PROJECT_SOURCE_DIR}/devel/esMulti/world3d.h -o ${PROJECT_SOURCE_DIR}/devel/esMulti/mworld3d.cc
                     DEPENDS ${PROJECT_SOURCE_DIR}/devel/esMulti/world3d.cc ${PROJECT_SOURCE_DIR}/devel/esMulti/world3d.cc
                     )

INCLUDE_DIRECTORIES(
        ${PROJECT_SOURCE_DIR}/recserver
        ${PROJECT_SOURCE_DIR}/flowserver
        ${PROJECT_SOURCE_DIR}/depthflowserver
	)

LINK_DIRECTORIES(
	)

ADD_EXECUTABLE( esMulti
		main.cc
		world3d.cc
 		mworld3d.cc
		proc.cc
		pointfilter.cc
		)

TARGET_LINK_LIBRARIES(esMulti
	ippi
	glut
        Qt3Support
        QtCore
        QtOpenGL
        QtGui
	pthread
	)

# install targets
INSTALL_TARGETS(${BIN_DIR} 
  esMulti
)
