PROJECT(Attention)

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP)
FIND_PACKAGE(OpenCV)
# conio wrapper/replacement
FIND_PACKAGE(SIMIO)
INCLUDE_DIRECTORIES(${SIMIO_INCLUDE_DIRS})
LINK_LIBRARIES(${SIMIO_LIBRARIES})

FILE(GLOB folder_header *.h)
FILE(GLOB folder_source *.cpp)
#FILE(GLOB sub_folder_header ./interface/*.h)
##FILE(GLOB sub_folder_source ./interface/*.cpp)


#include_directories (${SVN}lib/kinematics ${SVN}lib/vision/tracking )

#include_directories (./interface)

INCLUDE_DIRECTORIES(${OPENCV_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})
#INCLUDE_DIRECTORIES(${GSL_INCLUDE_DIR})

LINK_LIBRARIES(${OPENCV_LIBRARIES})
#LINK_LIBRARIES(${GSL_LIBRARIES})
#LINK_LIBRARIES(${GSLCBLAS_LIBRARIES})


ADD_EXECUTABLE(attention ${folder_source} ${folder_header})
