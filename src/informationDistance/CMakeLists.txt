FIND_PACKAGE(YARP)

PROJECT(InformationDistance)

ADD_EXECUTABLE(computeDistance ComputeDistance.cpp BinWindowMaxEntropy.h DistanceMatrix.h  InformationDistanceMatrix.h)

ADD_EXECUTABLE(relax relax.cpp)

