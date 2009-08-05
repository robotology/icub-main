PROJECT(stereovision)


FIND_PACKAGE(YARP)

FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OPENCV_INCLUDE_DIR})
LINK_DIRECTORIES(${OPENCV_LINK_DIRECTORIES})
LINK_LIBRARIES(${OPENCV_LIBRARIES})


INCLUDE_DIRECTORIES(include/ include/iCub /usr/local/include/opencv)


IF (WIN32 AND NOT CYGWIN)
  ADD_DEFINITIONS(-DWIN32 -D_WINDOWS)
ENDIF (WIN32 AND NOT CYGWIN)

IF(WIN32 AND CYGWIN)
  ADD_DEFINITIONS(-DCYGWIN)
ENDIF(WIN32 AND CYGWIN)


SET(CMAKE_CXX_FLAGS  "-DTRUE=1 -DFALSE=0 -O3 -Wall -Wno-unused-function")



IF(NOT WIN32)
ADD_DEFINITIONS(-DLINUX)
ENDIF(NOT WIN32)

ADD_DEFINITIONS(-DWITH_YARP -DNEW_VERSION)

SET(CMAKE_VERBOSE_MAKEFILE ON)


#SET(CMAKE_EXE_LINKER_FLAGS "-lstdc++ -lm -lcvaux -lopencv -lhighgui")
#=======
# this is very non-portable; inserted LINK_LIBRARIES(${OPENCV_LIBRARIES}) earlier which should have a similar effect



## one must check that YARP_COMM is consistent across paltforms
#IF(YARP_FOUND)
#SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
#ENDIF(YARP_FOUND)

INCLUDE_DIRECTORIES(include ../include /usr/local/include/opencv include/misc include/misc/MathLib include/iCub )


IF (USB_TEST)
ADD_DEFINITIONS(-DSTEREOVISION_MAIN)
SET (STEREO_VISION_SOURCES
   src/basicOpenCV.cpp	
    src/locator3D.cpp		
     src/colorTracker.cpp
     src/colordetect.cpp
     src/CameraV4L.cpp
     src/CameraV4LPWC.cpp
     src/Camera.cpp
     src/distortionCorrector.cpp
     src/Timer.cpp
     src/blobTracker.cpp
     src/gmmTracker.cpp
     src/visionServer.cpp
     src/pd_kalman.cpp	
     src/stereovision.cpp
)
ELSE(USB_TEST)
ADD_DEFINITIONS(-DICUB -DYARP_IMAGE_GRABBER)
# uncomment this you are using opencv 0.9.7
#ADD_DEFINITIONS(-DICUB -DYARP_IMAGE_GRABBER -DOPENCV_0_9_7)
SET (STEREO_VISION_SOURCES
     src/basicOpenCV.cpp
     src/locator3D.cpp
     src/colordetect.cpp
     src/colorTracker.cpp
     src/Camera.cpp
     src/distortionCorrector.cpp
     src/Timer.cpp
     src/blobTracker.cpp
     src/gmmTracker.cpp
     src/visionServer.cpp
     src/pd_kalman.cpp	
     src/stereovision.cpp
     src/stereoVisualTracker.cpp
)
ENDIF(USB_TEST)

SET (VISUAL_FLOW_SOURCES
    src/absorient.cpp
    src/visualRotation.cpp
    src/basicOpenCV.cpp
    src/quaternionflow.cpp   
    src/Vector.cpp
    src/visionServer.cpp
    src/visualRotationModule.cpp
    src/opticalflow.cpp
)


IF (NOT WIN32)
   LINK_LIBRARIES(ncurses)
ENDIF (NOT WIN32)



ADD_EXECUTABLE(stereoVisualTracker ${STEREO_VISION_SOURCES})
ADD_EXECUTABLE(visualRotation ${VISUAL_FLOW_SOURCES})

