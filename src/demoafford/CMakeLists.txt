
SET(KEYWORD "demoAff")

# Start a project.
PROJECT(${KEYWORD})

SET(YARP_DIR "$ENV{YARP_ROOT}" CACHE LOCATION "where is yarp?")
SET(SVN "/home/luis/vislab")

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(GSL REQUIRED)
FIND_PACKAGE(OpenCV)


FILE(GLOB folder_header ./include/iCub/*.h )
FILE(GLOB folder_source ./src/*.cpp )
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(./include)


INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})
INCLUDE_DIRECTORIES(${ICUB_DIR}/src/kinematics/include)
INCLUDE_DIRECTORIES(${GSL_INCLUDE_DIR})
#INCLUDE_DIRECTORIES(${SVN}/lib/kinematics)

INCLUDE_DIRECTORIES(/usr/local/include)
INCLUDE_DIRECTORIES(/usr/local/include/opencx)


LINK_LIBRARIES(${ICUB_DIR}/lib/libkinematics.a)
#LINK_LIBRARIES(${OPENCV_LIBRARIES})
LINK_LIBRARIES(${GSL_LIBRARIES})
LINK_LIBRARIES(${GSLCBLAS_LIBRARIES})

ADD_EXECUTABLE(demoAff ${folder_source} ${folder_header})
TARGET_LINK_LIBRARIES(demoAff /usr/local/lib/libpnl.so /usr/local/lib/libcxcore.a)
