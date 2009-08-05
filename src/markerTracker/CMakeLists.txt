##############################################
#        CMakeLists for making Module
#        ****************************
#                    markerTracker
#
#	13 / 07 / 2007, M. C. Bucko, Genova
##############################################

PROJECT(markerTracker)

FIND_PACKAGE(ICUB QUIET)

SET(Project_DIR ${CMAKE_SOURCE_DIR})
FIND_PACKAGE(YARP)
FIND_PACKAGE(OpenCV)

SET(header_path "${Project_DIR}/include")

#---------------------------------------------
# parse to collect source and header files
#---------------------------------------------

# catching all code files
FILE(GLOB codefiles src/*.cpp)

# catching all header files
FILE(GLOB_RECURSE headerfiles ${header_path}/*.h)

#---------------------------------------------
# adding the libraries & includes (to MSP6.0)
#---------------------------------------------

INCLUDE_DIRECTORIES(${OPENCV_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${header_path})
LINK_LIBRARIES(${OPENCV_LIBRARIES})

#---------------------------------------------
# grouping
#---------------------------------------------

SOURCE_GROUP("Header Files" FILES ${headerfiles})
SOURCE_GROUP("Source Files" FILES ${codefiles})

#---------------------------------------------
# making the executable
#---------------------------------------------

ADD_EXECUTABLE(${PROJECT_NAME} ${codefiles} ${headerfiles})

