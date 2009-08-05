PROJECT(SoundLoc)
#SET(YARP_DIR "c:/yarp_root/yarp2/" CACHE LOCATION "where is yarp?")
#SET(SVN "D:/svn/")

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP)
FIND_PACKAGE(OpenCV)
FIND_PACKAGE(GSL)

# replace CONIO
FIND_PACKAGE(SIMIO REQUIRED)
INCLUDE_DIRECTORIES(${SIMIO_INCLUDE_DIRS})
LINK_LIBRARIES(${SIMIO_LIBRARIES})

FILE(GLOB folder_header *.h)
FILE(GLOB folder_source *.cpp)
FILE(GLOB sub_folder_header ./interface/*.h)
FILE(GLOB sub_folder_source ./interface/*.cpp)


#include_directories (${SVN}lib/kinematics ${SVN}lib/vision/tracking ./interface)

include_directories (./interface)

INCLUDE_DIRECTORIES(${OPENCV_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})
INCLUDE_DIRECTORIES(${GSL_INCLUDE_DIR})

LINK_LIBRARIES(${OPENCV_LIBRARIES})
LINK_LIBRARIES(${GSL_LIBRARIES})
LINK_LIBRARIES(${GSLCBLAS_LIBRARIES})


ADD_EXECUTABLE(SoundLoc ${folder_source} ${folder_header} ${sub_folder_header} ${sub_folder_source})
