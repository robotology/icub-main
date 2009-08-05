PROJECT(canLoader)

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP)

IF (NOT GtkPlus_FOUND)
	FIND_PACKAGE(GtkPlus)
ENDIF (NOT GtkPlus_FOUND)

IF (GtkPlus_FOUND)
	ADD_DEFINITIONS(${GTKPLUS_C_FLAGS})
	LINK_LIBRARIES(${GTKPLUS_LINK_FLAGS})

	INCLUDE_DIRECTORIES(${GTKPLUS_INCLUDE_DIR})

	SET(folder_source main.cpp driver.cpp downloader.cpp)
	SET(folder_header driver.h downloader.h)
	SOURCE_GROUP("Source Files" FILES ${folder_source})
	SOURCE_GROUP("Header Files" FILES ${folder_header})

	IF (WIN32 AND NOT CYGWIN)
    		FIND_LIBRARY(libntcan ntcan ${ICUB_DIR}/src/modules/esdCanAPi/winnt/lib/)
    		LINK_LIBRARIES(${libntcan})
    		INCLUDE_DIRECTORIES(${ICUB_DIR}/src/modules/esdCanAPi/winnt/include) 
		#    INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../common)
 
 		  #win32 here creates an executable without console
    		ADD_EXECUTABLE(canLoader WIN32 ${folder_source} ${folder_header})
		ELSE (WIN32 AND NOT CYGWIN)
		FIND_LIBRARY(libntcan ntcan ${ICUB_DIR}/src/modules/esdCanAPi/linux/lib/)
		LINK_LIBRARIES(${libntcan})
    		INCLUDE_DIRECTORIES(${ICUB_DIR}/src/modules/esdCanAPi/linux/include) 
		#    INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../common)
	       ADD_EXECUTABLE(canLoader ${folder_source} ${folder_header})
	ENDIF (WIN32 AND NOT CYGWIN)

	INSTALL_TARGETS(/bin canLoader) 
ELSE (GTKPlus_FOUND)
    MESSAGE(ERROR "gtk+ not found, won't compile canLoader")
ENDIF (GtkPlus_FOUND)
