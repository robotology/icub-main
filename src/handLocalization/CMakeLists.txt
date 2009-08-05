##############################################
#        CMakeLists for making Module
#        ****************************
#                    HALO
#
#	31.July 2006, M. Bucko, Genova
##############################################

PROJECT(handlocalization)

SET(YARP_DIR "$ENV{YARP_ROOT}" CACHE LOCATION "where is the yarp build directory?")

SET(Project_DIR ${CMAKE_SOURCE_DIR})
FIND_PACKAGE(YARP)
FIND_PACKAGE(OpenCV)

SET(header_path "${Project_DIR}/include")
#SET(opencv_library "${OPENCV_DIR}/cv/include,${OPENCV_DIR}/cxcore/include,${OPENCV_DIR}/cvaux/include,${OPENCV_DIR}/otherlibs/highgui")
#SET(libHALO_ImgProc "${Project_DIR}/src/libHALO_ImgProc/include")
SET(EXECUTABLE_OUTPUT_PATH ${Project_DIR})

MARK_AS_ADVANCED(EXECUTABLE_OUTPUT_PATH)

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
#INCLUDE_DIRECTORIES(${libHALO_ImgProc})
#INCLUDE_DIRECTORIES(${opencv_library})
#LINK_DIRECTORIES(${Project_DIR}/lib ${Project_DIR}/lib/Debug ${Project_DIR}/lib/Release)
#LINK_DIRECTORIES(${Project_DIR}/lib)

LINK_LIBRARIES(${OPENCV_LIBRARIES})

#---------------------------------------------
# grouping
#---------------------------------------------

#SOURCE_GROUP("Header Files" FILES ${headerfiles})
#SOURCE_GROUP("Source Files" FILES ${codefiles})

#---------------------------------------------
# making the executable
#---------------------------------------------

ADD_EXECUTABLE(${PROJECT_NAME} ${codefiles} ${headerfiles})

