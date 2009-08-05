PROJECT(robotMotorGui)

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

# check if we have already found GTK
IF(NOT GtkPlus_FOUND)
	FIND_PACKAGE(GtkPlus)
ENDIF (NOT GtkPlus_FOUND)

IF (GtkPlus_FOUND)
	ADD_DEFINITIONS(${GTKPLUS_C_FLAGS})
	LINK_LIBRARIES(${GTKPLUS_LINK_FLAGS})

	INCLUDE_DIRECTORIES(${GTKPLUS_INCLUDE_DIR})

	FILE(GLOB_RECURSE folder_source src/*.cpp)
	FILE(GLOB_RECURSE folder_header src/include/*.h)
	SOURCE_GROUP("Source Files" FILES ${folder_source})
	SOURCE_GROUP("Header Files" FILES ${folder_header})

	ADD_EXECUTABLE(robotMotorGui ${folder_source} ${folder_header})

	INSTALL(TARGETS robotMotorGui RUNTIME DESTINATION bin)

	# organization of configuration files in installations
	SET(appdir app/default/conf)

	# configuration files
	INSTALL(FILES robotMotorGui.ini DESTINATION ${appdir})
ELSEIF (GtkPlux_FOUND)
	MESSAGE("gtk+ not found, won't compile robotMotorGui")
ENDIF (GtkPlus_FOUND)
#
#
