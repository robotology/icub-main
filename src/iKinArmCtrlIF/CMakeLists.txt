SET(PROJECTNAME iKinArmCtrlIF)
PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

IF(USE_ICUB_MOD)
	FIND_PACKAGE(icubmod)
	IMPORT_DEVICES(${ICUB_DIR}/src/${PROJECTNAME}/drivers.h icubmod)
	SET(folder_header drivers.h)
ENDIF(USE_ICUB_MOD)

SET(folder_source main.cpp)

SOURCE_GROUP("Header Files" FILES ${folder_header})
SOURCE_GROUP("Source Files" FILES ${folder_source})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_header} ${folder_source})
INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)




