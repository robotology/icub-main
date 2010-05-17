PREPARE_DEVICE(xsensmtx TYPE XSensMTx INCLUDE XSensMTx.h WRAPPER inertial)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}
                    ${YARP_INCLUDE_DIRS})

ADD_LIBRARY(xsensmtx MTComm.cpp  MTComm.h  XSensMTx.cpp  XSensMTx.h)

target_link_libraries(xsensmtx ${YARP_LIBRARIES})

icub_export_library(xsensmtx "" include/iCub "")