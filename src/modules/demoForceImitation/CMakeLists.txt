SET(PROJECTNAME demoForceImitation)
PROJECT(${PROJECTNAME})

FILE(GLOB_RECURSE folder_header *.h)
FILE(GLOB_RECURSE folder_source *.cpp)
SOURCE_GROUP("Header Files" FILES ${folder_header})
SOURCE_GROUP("Source Files" FILES ${folder_source})


INCLUDE_DIRECTORIES(${iCubDev_INCLUDE_DIRS}
	            ${ctrlLib_INCLUDE_DIRS}
		    ${YARP_INCLUDE_DIRS}
		    ${skinDynLib_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_header} ${folder_source})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ctrlLib
				     icubmod
				     ${YARP_LIBRARIES}
				     skinDynLib)

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)


