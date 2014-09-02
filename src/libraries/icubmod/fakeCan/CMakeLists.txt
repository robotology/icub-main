# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PREPARE_DEVICE(fakecan  TYPE FakeCan
                        INCLUDE fakeCan.h)


IF (NOT SKIP_fakecan)
    include_directories(${YARP_INCLUDE_DIRS})
    yarp_add_plugin(fcan fakeCan.cpp fakeBoard.cpp fakeCan.h fakeBoard.h fbCanBusMessage.h msgList.h)
    target_link_libraries(fcan ${YARP_LIBRARIES})
    
    icub_export_library(fcan)
ENDIF (NOT SKIP_fakecan)
