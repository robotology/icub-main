cmake_minimum_required(VERSION 2.6)

SET(PROJECTNAME learningMachine)

SET(LM_TRAIN_EXEC lmtrain)
SET(LM_PREDICT_EXEC lmpredict)
SET(LM_TRANSFORM_EXEC lmtransform)
SET(LM_TEST_EXEC lmtest)
SET(LM_MERGE_EXEC lmmerge)

PROJECT(${PROJECTNAME})

# Source code Groups
SET(LM_MODULE_SRC 
    src/IMachineLearnerModule.cpp )
SET(LM_EVENT_SRC 
    src/DispatcherManager.cpp
    src/EventDispatcher.cpp 
    src/IEvent.cpp
    src/IEventListener.cpp 
    src/IPortEventListener.cpp
    src/PredictEvent.cpp 
    src/PredictEventListener.cpp
    src/TrainEvent.cpp 
    src/TrainEventListener.cpp )

SET(LM_HEADER 
    include/iCub/learningMachine/DispatcherManager.h
    include/iCub/learningMachine/EventDispatcher.h
    include/iCub/learningMachine/EventListenerCatalogue.h
    include/iCub/learningMachine/EventListenerFactory.h
    include/iCub/learningMachine/FileReaderT.h
    include/iCub/learningMachine/IEvent.h
    include/iCub/learningMachine/IEventListener.h
    include/iCub/learningMachine/IMachineLearnerModule.h
    include/iCub/learningMachine/IPortEventListener.h
    include/iCub/learningMachine/PredictEvent.h
    include/iCub/learningMachine/PredictEventListener.h
    include/iCub/learningMachine/PredictModule.h
    include/iCub/learningMachine/TrainEvent.h
    include/iCub/learningMachine/TrainEventListener.h
    include/iCub/learningMachine/TrainModule.h
    include/iCub/learningMachine/TransformModule.h )

# Declare groups of source and header files -- makes things pretty in MSVC.
SOURCE_GROUP("Source Files" FILES ${LM_MODULE_SRC} 
                                  ${LM_EVENT_SRC} )
SOURCE_GROUP("Header Files" FILES ${LM_HEADER})


# add YARP include directories
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})
# add ACE dependence (temporary workaround)
INCLUDE_DIRECTORIES(${ACE_INCLUDE_DIRS})
# add learningMachine library include directories
INCLUDE_DIRECTORIES(${learningMachine_INCLUDE_DIRS})
# add our include files into our compiler's search path.
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)


ADD_EXECUTABLE(${LM_TRAIN_EXEC} ${LM_HEADER} ${LM_MODULE_SRC} ${LM_EVENT_SRC} src/TrainModule.cpp src/PredictModule.cpp src/bin/train.cpp)
ADD_EXECUTABLE(${LM_PREDICT_EXEC} ${LM_HEADER} ${LM_MODULE_SRC} ${LM_EVENT_SRC} src/PredictModule.cpp src/bin/predict.cpp)
ADD_EXECUTABLE(${LM_TRANSFORM_EXEC} ${LM_HEADER} ${LM_MODULE_SRC} ${LM_EVENT_SRC} src/TransformModule.cpp src/bin/transform.cpp)
ADD_EXECUTABLE(${LM_TEST_EXEC} src/bin/test.cpp)
ADD_EXECUTABLE(${LM_MERGE_EXEC} src/bin/merge.cpp)

TARGET_LINK_LIBRARIES(${LM_TRAIN_EXEC} learningMachine ${YARP_LIBRARIES})
TARGET_LINK_LIBRARIES(${LM_PREDICT_EXEC} learningMachine ${YARP_LIBRARIES})
TARGET_LINK_LIBRARIES(${LM_TRANSFORM_EXEC} learningMachine ${YARP_LIBRARIES})
TARGET_LINK_LIBRARIES(${LM_TEST_EXEC} ${YARP_LIBRARIES})
TARGET_LINK_LIBRARIES(${LM_MERGE_EXEC} ${YARP_LIBRARIES})


INSTALL(TARGETS ${LM_TRAIN_EXEC} ${LM_PREDICT_EXEC} ${LM_TRANSFORM_EXEC} ${LM_TEST_EXEC} ${LM_MERGE_EXEC} DESTINATION bin)

