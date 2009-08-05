PROJECT(neckNeuralController)

SET(YARP_DIR "$ENV{YARP_ROOT}" CACHE LOCATION "where is yarp?")
SET(ICUB_DIR "$ENV{ICUB_ROOT}" CACHE LOCATION "where is icub root?")

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP)

FIND_PACKAGE(YARPMATH REQUIRED)
FIND_PACKAGE(RFWR REQUIRED)

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})

INCLUDE_DIRECTORIES(${RFWR_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${YARPMATH_INCLUDE_DIRS})
#INCLUDE_DIRECTORIES(${ICUB_DIR}/src/yarpmath)

#INCLUDE_DIRECTORIES(${ICUB_DIR}/src/rfwr)
#INCLUDE_DIRECTORIES(${ICUB_DIR}/src/yarpmath)

SOURCE_GROUP("Source Files" FILES symbolicModel.cpp)
SOURCE_GROUP("Header Files" FILES symbolicModel.h)

ADD_LIBRARY(symbolicModel STATIC symbolicModel.cpp symbolicModel.h)

LINK_LIBRARIES(neckNeuralController symbolicModel)
LINK_LIBRARIES(neckNeuralController ${RFWR_LIBRARIES})
LINK_LIBRARIES(neckNeuralController ${YARPMATH_LIBRARIES})

# these were all windows specific
#LINK_LIBRARIES(neckNeuralController ${CMAKE_SOURCE_DIR}/Debug/symbolicModel.lib)
#LINK_LIBRARIES(neckNeuralController ${ICUB_DIR}/src/rfwr/release/rfwr.lib)
#LINK_LIBRARIES(neckNeuralController ${ICUB_DIR}/src/yarpmath/release/yarpmath.lib)


ADD_EXECUTABLE(neckNeuralController main.cpp)

#ADD_DEPENDENCIES(neckNeuralController symbolicModel) # not needed


