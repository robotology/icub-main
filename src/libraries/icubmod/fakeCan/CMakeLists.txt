IF (COMPILE_DEVICE_LIBRARY)
  PREPARE_DEVICE(fakecan TYPE FakeCan
    INCLUDE fakeCan.h 
    WRAPPER "")
ENDIF (COMPILE_DEVICE_LIBRARY)

IF (NOT SKIP_fcan)
    FIND_PACKAGE(YARP REQUIRED)
    ADD_LIBRARY(fcan fakeCan.cpp fakeBoard.cpp fakeCan.h fakeBoard.h fbCanBusMessage.h msgList.h)
ENDIF (NOT SKIP_fcan)
