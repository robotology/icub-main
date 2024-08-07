# Copyright: (C) 2010 RobotCub Consortium
# Authors:  Arjan Gijsberts
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME learningMachine)

find_package(YARP)

set(LM_LIB_DIR ../../library/standalone)

set(LM_TRAIN_EXEC lmtrain)
set(LM_PREDICT_EXEC lmpredict)
set(LM_TRANSFORM_EXEC lmtransform)
set(LM_TEST_EXEC lmtest)
set(LM_MERGE_EXEC lmmerge)

PROJECT(${PROJECTNAME})

# Source code Groups
set(LM_MODULE_SRC
    ../src/IMachineLearnerModule.cpp )
set(LM_EVENT_SRC
    ../src/DispatcherManager.cpp
    ../src/EventDispatcher.cpp
    ../src/IEvent.cpp
    ../src/IEventListener.cpp
    ../src/IPortEventListener.cpp
    ../src/PredictEvent.cpp
    ../src/PredictEventListener.cpp
    ../src/TrainEvent.cpp
    ../src/TrainEventListener.cpp )

set(LM_HEADER
    ../include/iCub/learningMachine/DispatcherManager.h
    ../include/iCub/learningMachine/EventDispatcher.h
    ../include/iCub/learningMachine/EventListenerCatalogue.h
    ../include/iCub/learningMachine/EventListenerFactory.h
    ../include/iCub/learningMachine/FileReaderT.h
    ../include/iCub/learningMachine/IEvent.h
    ../include/iCub/learningMachine/IEventListener.h
    ../include/iCub/learningMachine/IMachineLearnerModule.h
    ../include/iCub/learningMachine/IPortEventListener.h
    ../include/iCub/learningMachine/PredictEvent.h
    ../include/iCub/learningMachine/PredictEventListener.h
    ../include/iCub/learningMachine/PredictModule.h
    ../include/iCub/learningMachine/TrainEvent.h
    ../include/iCub/learningMachine/TrainEventListener.h
    ../include/iCub/learningMachine/TrainModule.h
    ../include/iCub/learningMachine/TransformModule.h )

# Declare groups of source and header files -- makes things pretty in MSVC.
source_group("Source Files" FILES ${LM_MODULE_SRC}
                                  ${LM_EVENT_SRC} )
source_group("Header Files" FILES ${LM_HEADER})


# add ACE dependence (temporary workaround)
include_directories(${ACE_INCLUDE_DIRS})
# add learningMachine library include directories
include_directories(${LM_LIB_DIR}/../include)
# add our include files into our compiler's search path.
include_directories(${PROJECT_SOURCE_DIR}/../include)

link_directories(${LINK_DIRECTORIES} ${LM_LIB_DIR})
set(LM_LIB learningMachine)



add_executable(${LM_TRAIN_EXEC} ${LM_HEADER} ${LM_MODULE_SRC} ${LM_EVENT_SRC} ../src/TrainModule.cpp ../src/PredictModule.cpp ../src/bin/train.cpp)
add_executable(${LM_PREDICT_EXEC} ${LM_HEADER} ${LM_MODULE_SRC} ${LM_EVENT_SRC} ../src/PredictModule.cpp ../src/bin/predict.cpp)
add_executable(${LM_TRANSFORM_EXEC} ${LM_HEADER} ${LM_MODULE_SRC} ${LM_EVENT_SRC} ../src/TransformModule.cpp ../src/bin/transform.cpp)
add_executable(${LM_TEST_EXEC} ../src/bin/test.cpp)
add_executable(${LM_MERGE_EXEC} ../src/bin/merge.cpp)

target_link_libraries(${LM_TRAIN_EXEC} ${LM_LIB} ${YARP_LIBRARIES})
target_link_libraries(${LM_PREDICT_EXEC} ${LM_LIB} ${YARP_LIBRARIES})
target_link_libraries(${LM_TRANSFORM_EXEC} ${LM_LIB} ${YARP_LIBRARIES})
target_link_libraries(${LM_TEST_EXEC} ${YARP_LIBRARIES})
target_link_libraries(${LM_MERGE_EXEC} ${YARP_LIBRARIES})


install(TARGETS ${LM_TRAIN_EXEC} ${LM_PREDICT_EXEC} ${LM_TRANSFORM_EXEC} ${LM_TEST_EXEC} ${LM_MERGE_EXEC} DESTINATION bin)
