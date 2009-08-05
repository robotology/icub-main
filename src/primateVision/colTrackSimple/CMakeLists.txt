ADD_DEFINITIONS(-O3)


INCLUDE_DIRECTORIES(
        ${PROJECT_SOURCE_DIR}/utils/convert_rgb
	${PROJECT_SOURCE_DIR}/utils/configfile
	${PROJECT_SOURCE_DIR}/utils/display
	${PROJECT_SOURCE_DIR}/utils/multiFrameViewer
	)

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/convert_rgb
	${PROJECT_SOURCE_DIR}/utils/display
	${PROJECT_SOURCE_DIR}/utils/multiFrameViewer
	)

ADD_EXECUTABLE( colTrackSimple
		main.cc 
		)

TARGET_LINK_LIBRARIES(colTrackSimple
	QtCore
        QtGui
	ippi
	ippcv
	ippcc
	convert_rgb
	display
	multiframeviewer
        pthread
	)

# install targets
INSTALL_TARGETS(${BIN_DIR} 
  colTrackSimple
  )
