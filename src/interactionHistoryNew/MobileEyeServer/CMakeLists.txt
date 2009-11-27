PROJECT(mobileeye)
 FIND_PACKAGE(YARP REQUIRED)
# FIND_PACKAGE(GTKMM REQUIRED)
# FIND_PACKAGE(GLADEMM REQUIRED)
# INCLUDE_DIRECTORIES(${GTKMM_INCLUDE_DIRS})
# INCLUDE_DIRECTORIES(${GLADEMM_INCLUDE_DIRS})
# SET(LIBS ${LIBS} ${GTKMM_LIBRARIES} ${GLADEMM_LIBRARIES})
 ADD_LIBRARY(mobileeye MobileEyeServer.h 
MobileEye.cpp MobileEye.h 
SerialThread.cpp SerialThread.h 
ASLSerialOutDecode.cpp ASLSerialOutDecode.h 
ASLCircularBuffer.cpp ASLCircularBuffer.h)
 LINK_LIBRARIES(mobileeye)
 ADD_EXECUTABLE(example TestMobileEye.cpp)
#ADD_EXECUTABLE(mobeyemodule MobileEyeModule.cpp MobileEyeModule.h)
#TARGET_LINK_LIBRARIES(guiexample ${LIBS})



