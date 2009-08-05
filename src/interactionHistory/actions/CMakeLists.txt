SET(KEYWORD "ihaActionsLib")

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

# Source code for salience remote library
FILE(GLOB source_lib 
    src/Actions.cpp
)
FILE(GLOB header_lib
    include/iCub/iha/Actions.h
    include/iCub/iha/Sequence.h
)

LINK_LIBRARIES(
	${IHADEBUGLIB_LIBRARIES}
)

# Set up library
ADD_LIBRARY(${KEYWORD} ${source_lib} ${header_lib})
INSTALL_TARGETS(/lib ${KEYWORD})

