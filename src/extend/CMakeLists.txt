
PROJECT(extend)

FIND_PACKAGE(ICUB)

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(Boost REQUIRED)

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
LINK_LIBRARIES(${Boost_LIBRARIES})

ADD_EXECUTABLE(extend main.cpp fetch.cpp fetch.h yarp.h yarp.cpp ConstantSequence.cpp SymbolicSequence.cpp PiecewiseSequence.cpp Sequence.cpp IUnit.cpp Tests.cpp)
