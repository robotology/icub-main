PROJECT(iCubInterfaceGui)

ADD_DEFINITIONS(${GTKMM_C_FLAGS})
INCLUDE_DIRECTORIES(${GTKMM_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(iCubInterfaceGui 
               iCubBoardChannel.h 
               iCubBoardChannel.cpp 
               iCubBoard.h 
               iCubBoard.cpp 
               iCubNetwork.h 
               iCubNetwork.cpp
               iCUbInterfaceGuiServer.h
	       iCUbInterfaceGuiServer.cpp
               main.cpp)


ADD_EXECUTABLE(iCubInterfaceGuiClient 
               iCubBoardChannel.h 
               iCubBoardChannel.cpp 
               iCubBoard.h 
               iCubBoard.cpp 
               iCubNetwork.h 
               iCubNetwork.cpp
               iCubBoardChannelGui.h 
               iCubBoardChannelGui.cpp 
               iCubBoardGui.h 
               iCubBoardGui.cpp 
               iCubNetworkGui.h 
               iCubNetworkGui.cpp
               iCUbInterfaceGuiServer.h
	       iCUbInterfaceGuiServer.cpp
               iCUbInterfaceGuiClient.h
	       iCUbInterfaceGuiClient.cpp
               mainClient.cpp)

TARGET_LINK_LIBRARIES(iCubInterfaceGui ${YARP_LIBRARIES})
TARGET_LINK_LIBRARIES(iCubInterfaceGuiClient 
                      ${GTKMM_LIBRARIES} 
                      ${YARP_LIBRARIES} 
                      ${GTHREAD_LIBRARIES}
)

IF (WIN32 AND NOT CYGWIN)
	SET_TARGET_PROPERTIES(iCubInterfaceGui PROPERTIES COMPILE_FLAGS /wd4250)
	SET_TARGET_PROPERTIES(iCubInterfaceGuiClient PROPERTIES COMPILE_FLAGS /wd4250)
ENDIF (WIN32 AND NOT CYGWIN)

INSTALL_TARGETS(/bin iCubInterfaceGui)


