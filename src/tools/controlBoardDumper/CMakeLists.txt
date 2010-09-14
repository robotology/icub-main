PROJECT(controlBoardDumper)

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

include_directories(${YARP_INCLUDE_DIRS})
ADD_EXECUTABLE(controlBoardDumper ${folder_source} ${folder_header})
target_link_libraries(controlBoardDumper ${YARP_LIBRARIES})
INSTALL(TARGETS controlBoardDumper DESTINATION bin)

