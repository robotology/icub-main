PROJECT(learner)

#IF (UNIX)
#  ADD_DEFINITIONS(-Wall)
#ENDIF (UNIX)

FIND_PACKAGE(YARP)

# Find source code files
FILE(GLOB HEADERS *.h)
#FILE(GLOB CODE *.cpp)
SET(CODE dataSet.cpp libsvm.cpp learningMachine.cpp 
         libsvmLearningMachine.cpp normaliser.cpp)

# Lay the files out in folders in environments where that makes sense
# This is completely optional
SOURCE_GROUP("Source Files" FILES ${CODE} main.cpp)
SOURCE_GROUP("Header Files" FILES ${HEADERS})

ADD_EXECUTABLE(learner ${HEADERS} ${CODE} main.cpp)
ADD_EXECUTABLE(testerLearner ${HEADERS} test.cpp)
INSTALL_TARGETS(/bin learner testerLearner)

