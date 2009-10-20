ADD_DEFINITIONS(-DQT_THREAD_SUPPORT -O3 -DQT3_SUPPORT -D_LINUX)


ADD_LIBRARY(	timing
		SHARED
		timing.cc
		)

# install targets
INSTALL_TARGETS(${LIB_DIR} 
  timing
  )
