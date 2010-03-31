PROJECT(test_gyro)

FILE(GLOB folder_header *.h)
FILE(GLOB folder_source *.cpp)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})
ADD_EXECUTABLE(test_gyro ${folder_source} ${folder_header})


