SET(KEYWORD "rw_utils")
PROJECT(${KEYWORD})

SET(IHADEBUGLIB_DIR ${ICUB_DIR}/src/interactionHistory/debug/)

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(IHADEBUGLIB REQUIRED)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
	${IHADEBUGLIB_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/../include
)

LINK_LIBRARIES(
	${IHADEBUGLIB_LIBRARIES}
)
ADD_EXECUTABLE(ihaPortReader PortReader.cpp)
ADD_EXECUTABLE(ihaSensorsFileReader SensorsFileReader.cpp ../include/iCub/iha/file_utils.h)
ADD_EXECUTABLE(ihaSensorsFileWriter SensorsFileWriter.cpp)

INSTALL_TARGETS(/bin ihaPortReader)
INSTALL_TARGETS(/bin ihaSensorsFileReader)
INSTALL_TARGETS(/bin ihaSensorsFileWriter)
