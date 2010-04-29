PROJECT(canLoader20)

IF (GtkPlus_FOUND)
	FILE(GLOB folder_source *.cpp)
	FILE(GLOB folder_header *.h)
	SOURCE_GROUP("Source Files" FILES ${folder_source})
	SOURCE_GROUP("Header Files" FILES ${folder_header})

	ADD_DEFINITIONS(${GTKPLUS_C_FLAGS})
	include_directories(${GTKPLUS_INCLUDE_DIR})
	include_directories(${canLoaderLib_INCLUDE_DIRS})
	include_directories(${YARP_INCLUDE_DIRS})

	ADD_EXECUTABLE(canLoader20 ${folder_source} ${folder_header})
	target_link_libraries(canLoader20 ${GTKPLUS_LINK_FLAGS})
	target_link_libraries(canLoader20 canLoaderLib)
	target_link_libraries(canLoader20 ${YARP_LIBRARIES})
	target_link_libraries(canLoader20 icubmod)
#	TARGET_LINK_DIRECTORIES(canLoader20 ${GTKPLUS_LIBRARY_DIR})

	INSTALL(TARGETS canLoader20 DESTINATION bin)
ELSE (GTKPlus_FOUND)
    MESSAGE(ERROR "gtk+ not found, won't compile canLoader20")
ENDIF (GtkPlus_FOUND)
