IF(WIN32)
cmake_minimum_required(VERSION 2.6)
ELSE(WIN32)
cmake_minimum_required(VERSION 2.4)
ENDIF(WIN32)

PROJECT(CrawlPlanner)

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h )
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

ADD_EXECUTABLE(CrawlPlanner ${folder_source} ${folder_header})

