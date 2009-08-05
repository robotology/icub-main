ADD_DEFINITIONS(-DQT_THREAD_SUPPORT -O3)

SUBDIRS(testmulticlass)

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/coord
	)

ADD_LIBRARY( multiclass
		graph.cpp
		maxflow.cpp
		multiclass.cpp
		)

TARGET_LINK_LIBRARIES(multiclass
	ippi
	ippcv
	)



# install targets
INSTALL_TARGETS(${LIB_DIR} 
  multiclass
  )
