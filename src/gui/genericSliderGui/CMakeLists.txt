PROJECT(genericSliderGui)

FIND_PACKAGE(ICUB REQUIRED)
FIND_PACKAGE(YARP REQUIRED)

SET(NESTED_BUILD TRUE)

IF(USE_ICUB_MOD)
  ADD_DEFINITIONS(-DUSE_ICUB_MOD)
  FIND_PACKAGE(icubmod)
  IMPORT_DEVICES(${ICUB_DIR}/src/canLoader/canLoader2.0/canLoader/drivers.h icubmod)
ENDIF (USE_ICUB_MOD)

IF (NOT GtkPlus_FOUND)
	FIND_PACKAGE(GtkPlus)
ENDIF (NOT GtkPlus_FOUND)

IF (GtkPlus_FOUND)
	ADD_DEFINITIONS(${GTKPLUS_C_FLAGS})
	LINK_LIBRARIES(${GTKPLUS_LINK_FLAGS})

	INCLUDE_DIRECTORIES(${GTKPLUS_INCLUDE_DIR})

	FILE(GLOB folder_source *.cpp)
	FILE(GLOB folder_header *.h)
	SOURCE_GROUP("Source Files" FILES ${folder_source})
	SOURCE_GROUP("Header Files" FILES ${folder_header})


	# Check if the following works on Linux as well, otherwise
	# do it only IF (WIN32 AND NOT CYGWIN), else
	# ADD_EXECUTABLE(genericSliderGui ${folder_source} ${folder_header})

	ADD_EXECUTABLE(genericSliderGui WIN32 ${folder_source} ${folder_header})

	#INSTALL_TARGETS(/bin cgenericSliderGui) 
	INSTALL(TARGETS genericSliderGui DESTINATION ${ICUB_DIR}/bin)
ELSE (GTKPlus_FOUND)
    MESSAGE(ERROR "gtk+ not found, won't compile genericSliderGui")
ENDIF (GtkPlus_FOUND)
