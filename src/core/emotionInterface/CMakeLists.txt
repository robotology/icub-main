SET(KEYWORD "emotionInterface")

PROJECT(${KEYWORD})

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}
	${YARP_INCLUDE_DIRS}
)

# Search for source code.
FILE(GLOB folder_source *.cpp *.cc *.c)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})


ADD_EXECUTABLE(${KEYWORD} ${folder_source} ${folder_header})
TARGET_LINK_LIBRARIES(${KEYWORD} ${YARP_LIBRARIES})

INSTALL(TARGETS ${KEYWORD} DESTINATION bin)