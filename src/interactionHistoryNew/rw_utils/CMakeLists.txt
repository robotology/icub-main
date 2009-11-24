SET(KEYWORD "rw_utils_new")
PROJECT(${KEYWORD})

SET(IHADEBUGLIB_DIR ${ICUB_DIR}/src/interactionHistoryNew/debug/)

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
ADD_EXECUTABLE(ihaNewPortReader PortReader.cpp)
ADD_EXECUTABLE(ihaNewSensorsFileReader SensorsFileReader.cpp ../include/iCub/iha/file_utils.h)
ADD_EXECUTABLE(ihaNewSensorsFileWriter SensorsFileWriter.cpp)

INSTALL_TARGETS(/bin ihaNewPortReader)
INSTALL_TARGETS(/bin ihaNewSensorsFileReader)
INSTALL_TARGETS(/bin ihaNewSensorsFileWriter)
