PREPARE_DEVICE(fakecan TYPE FakeCan
    INCLUDE fakeCan.h 
    WRAPPER "")


IF (NOT SKIP_fcan)
    include_directories(${YARP_INCLUDE_DIRS})
    ADD_LIBRARY(fcan fakeCan.cpp fakeBoard.cpp fakeCan.h fakeBoard.h fbCanBusMessage.h msgList.h)
    target_link_libraries(fcan ${YARP_LIBRARIES})
    
    icub_export_library(fcan "" include/iCub "")
ENDIF (NOT SKIP_fcan)
