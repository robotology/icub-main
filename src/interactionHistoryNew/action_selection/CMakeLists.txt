SET(KEYWORD "ihaNewActionSelection")

PROJECT(${KEYWORD})

SET(IHADEBUGLIB_DIR ${ICUB_DIR}/src/interactionHistoryNew/debug/)
SET(IHAACTIONSLIB_DIR ${ICUB_DIR}/src/interactionHistoryNew/actions/)

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(IHADEBUGLIB REQUIRED)
FIND_PACKAGE(IHAACTIONSLIB REQUIRED)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
	${IHADEBUGLIB_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/../include
	${IHAACTIONSLIB_INCLUDE_DIRS}
)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

LINK_LIBRARIES(
	${IHADEBUGLIB_LIBRARIES}
	${IHAACTIONSLIB_LIBRARIES}
)

# Set up our main executable.
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
    )
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)

INSTALL_TARGETS(/bin ${KEYWORD})

