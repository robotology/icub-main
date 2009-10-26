cmake_minimum_required(VERSION 2.6)

SET(PROJECTNAME learningMachine)

SET(LM_TRAIN_EXEC train)
SET(LM_PREDICT_EXEC predict)
SET(LM_TRANSFORM_EXEC transform)
SET(LM_TEST_EXEC test)
SET(LM_LIB ${PROJECTNAME})

PROJECT(${PROJECTNAME})

SET(CMAKE_CXX_FLAGS "-O3")

FIND_PACKAGE(YARP)

FIND_PACKAGE(ICUB QUIET)
IF (NOT ICUB_FOUND)
    MESSAGE (STATUS "ICUB repository not found; assuming distributed build.")
    MESSAGE (STATUS "If this is not what you want, set ICUB_DIR variable.")
ENDIF (NOT ICUB_FOUND)

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
SET(LM_MACHINE_SRC 
    src/DatasetRecorder.cpp
    src/DummyLearner.cpp 
    src/IFixedSizeLearner.cpp
    src/LSSVMLearner.cpp
#    src/MachinePortable.cpp 
    src/RLSLearner.cpp )
SET(LM_TRANSFORMER_SRC 
    src/FixedRangeScaler.cpp 
    src/IFixedSizeTransformer.cpp 
    src/IScaler.cpp 
    src/ITransformer.cpp 
    src/Normalizer.cpp 
    src/RandomFeature.cpp
    src/ScaleTransformer.cpp 
    src/Standardizer.cpp )
SET(LM_HEADER 
    include/iCub/DatasetRecorder.h
    include/iCub/DispatcherManager.h
    include/iCub/DummyLearner.h
    include/iCub/EventDispatcher.h
    include/iCub/EventListenerCatalogue.h
    include/iCub/EventListenerFactory.h
    include/iCub/FactoryT.h
    include/iCub/FixedRangeScaler.h
    include/iCub/IEvent.h
    include/iCub/IEventListener.h
    include/iCub/IFixedSizeLearner.h
    include/iCub/IFixedSizeTransformer.h
    include/iCub/IMachineLearner.h
    include/iCub/IMachineLearnerModule.h
    include/iCub/IPortEventListener.h
    include/iCub/IScaler.h
    include/iCub/ITransformer.h
    include/iCub/LSSVMAtlasLearner.h
    include/iCub/LSSVMLearner.h
    include/iCub/MachineCatalogue.h
    include/iCub/MachineFactory.h
    include/iCub/MachinePortable.h
    include/iCub/MultiLearnerT.h
    include/iCub/Normalizer.h
    include/iCub/PortableTriple.h
    include/iCub/PredictEvent.h
    include/iCub/PredictEventListener.h
    include/iCub/PredictModule.h
    include/iCub/RandomFeature.h
    include/iCub/RLSLearner.h
    include/iCub/ScaleTransformer.h
    include/iCub/Standardizer.h
    include/iCub/TrainEvent.h
    include/iCub/TrainEventListener.h
    include/iCub/TrainModule.h
    include/iCub/TransformerCatalogue.h
    include/iCub/TransformerFactory.h
    include/iCub/TransformModule.h )

# Declare groups of source and header files -- makes things pretty in MSVC.
#SOURCE_GROUP("Source Files" FILES ${folder_source})
#SOURCE_GROUP("Header Files" FILES ${folder_header})

SET(USE_LSSVMATLAS FALSE CACHE BOOL "Select true if Atlas version of LSSVM has to be compiled")
IF(USE_LSSVMATLAS)
    # add LSSVM Wrapper to Machines
    SET(LM_MACHINE_SRC ${LM_MACHINE_SRC} src/LSSVMAtlasLearner.cpp)
    ADD_DEFINITIONS(-DBUILD_LSSVMATLAS)
ENDIF(USE_LSSVMATLAS)

SET(CREATE_LM_LIBRARY FALSE CACHE BOOL "Select true to create a learningMachine library")
IF(CREATE_LM_LIBRARY)
    # create library to directly include machines and/or transformers, without modules
    ADD_LIBRARY(${LM_LIB} ${LM_MACHINE_SRC} ${LM_TRANSFORMER_SRC})
    INSTALL(TARGETS ${LM_LIB} DESTINATION lib)
ENDIF(CREATE_LM_LIBRARY)

ADD_EXECUTABLE(${LM_TRAIN_EXEC} ${LM_MODULE_SRC} ${LM_MACHINE_SRC} ${LM_EVENT_SRC} src/TrainModule.cpp src/PredictModule.cpp src/bin/train.cpp)
ADD_EXECUTABLE(${LM_PREDICT_EXEC} ${LM_MODULE_SRC} ${LM_MACHINE_SRC} ${LM_EVENT_SRC} src/PredictModule.cpp src/bin/predict.cpp)
ADD_EXECUTABLE(${LM_TRANSFORM_EXEC} ${LM_MODULE_SRC} ${LM_TRANSFORMER_SRC} ${LM_EVENT_SRC} src/TransformModule.cpp src/bin/transform.cpp)
ADD_EXECUTABLE(${LM_TEST_EXEC} src/bin/test.cpp)


# Add liblssvm library that uses Atlas
IF(USE_LSSVMATLAS)
    # Find the LibLSSVM package
    SET(LSSVM_DIR "../lssvm")
    FIND_PACKAGE(LSSVM REQUIRED)
    INCLUDE("${LSSVM_DIR}/LSSVMLibraryDepends.cmake")
    TARGET_LINK_LIBRARIES(${LM_TRAIN_EXEC} ${LSSVM_LIBRARIES})
    TARGET_LINK_LIBRARIES(${LM_PREDICT_EXEC} ${LSSVM_LIBRARIES})
    TARGET_LINK_LIBRARIES(${LM_TRAIN_EXEC} ${LSSVM_LIB_DEPENDS})
    TARGET_LINK_LIBRARIES(${LM_PREDICT_EXEC} ${LSSVM_LIB_DEPENDS})
    INCLUDE_DIRECTORIES(${LSSVM_INCLUDE_DIRS})
ENDIF(USE_LSSVMATLAS)


# Add our include files into our compiler's search path.
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

INSTALL(TARGETS ${LM_TRAIN_EXEC} ${LM_PREDICT_EXEC} ${LM_TRANSFORM_EXEC} DESTINATION bin)
INSTALL(FILES ${LM_HEADER} DESTINATION include/iCub)

