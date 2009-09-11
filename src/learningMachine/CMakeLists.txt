cmake_minimum_required(VERSION 2.6)

SET(PROJECTNAME MachineLearner)

SET(TRAIN_EXEC train)
SET(PREDICT_EXEC predict)
SET(TRANSFORM_EXEC transform)
SET(TEST_EXEC test)

PROJECT(${PROJECTNAME})

SET(CMAKE_CXX_FLAGS "-O3")

FIND_PACKAGE(YARP)

FIND_PACKAGE(ICUB QUIET)
IF (NOT ICUB_FOUND)
   MESSAGE (STATUS "ICUB repository not found; assuming distributed build.")
   MESSAGE (STATUS "If this is not what you want, set ICUB_DIR variable.")
ENDIF (NOT ICUB_FOUND)

# Find source code files
#FILE(GLOB folder_source src/*.cpp)
#FILE(GLOB folder_header include/iCub/*.h)

# Source code Groups
SET(MODULE_SRC src/IMachineLearnerModule.cpp)
SET(MACHINE_SRC src/IFixedSizeLearner.cpp src/MachinePortable.cpp src/DatasetRecorder.cpp src/DummyLearner.cpp src/LSSVMLearner.cpp src/RLSLearner.cpp)
SET(TRANSFORMER_SRC src/ITransformer.cpp src/IFixedSizeTransformer.cpp src/ScaleTransformer.cpp src/IScaler.cpp src/Standardizer.cpp src/Normalizer.cpp src/FixedRangeScaler.cpp src/RandomFeature.cpp)
#SET(EVENT_SRC

# Declare groups of source and header files -- makes things pretty in MSVC.
#SOURCE_GROUP("Source Files" FILES ${folder_source})
#SOURCE_GROUP("Header Files" FILES ${folder_header})

SET(USE_LSSVMATLAS FALSE CACHE BOOL "Select true if Atlas version of LSSVM has to be compiled")
IF(USE_LSSVMATLAS)
    # add LSSVM Wrapper to Machines
    SET(MACHINE_SRC ${MACHINE_SRC} src/LSSVMAtlasLearner.cpp)
    ADD_DEFINITIONS(-DBUILD_LSSVMATLAS)
ENDIF(USE_LSSVMATLAS)

#FIXME, now we need transformer_src and machine_src for both train_exec and predict_exec

ADD_EXECUTABLE(${TRAIN_EXEC} ${MODULE_SRC} ${MACHINE_SRC} ${TRANSFORMER_SRC} src/TrainModule.cpp src/PredictModule.cpp src/bin/train.cpp)
ADD_EXECUTABLE(${PREDICT_EXEC} ${MODULE_SRC} ${MACHINE_SRC} ${TRANSFORMER_SRC} src/PredictModule.cpp src/bin/predict.cpp)
ADD_EXECUTABLE(${TRANSFORM_EXEC} ${MODULE_SRC} ${TRANSFORMER_SRC} src/TransformModule.cpp src/bin/transform.cpp)
ADD_EXECUTABLE(${TEST_EXEC} src/bin/test.cpp)


# Add liblssvm library that uses Atlas
IF(USE_LSSVMATLAS)
    # Find the LibLSSVM package
    SET(LSSVM_DIR "../lssvm")
    FIND_PACKAGE(LSSVM REQUIRED)
    INCLUDE("${LSSVM_DIR}/LSSVMLibraryDepends.cmake")
    #SET(LSSVM_LIB_DEPENDS ${lssvm_LIB_DEPENDS})
    TARGET_LINK_LIBRARIES(${TRAIN_EXEC} ${LSSVM_LIBRARIES})
    TARGET_LINK_LIBRARIES(${PREDICT_EXEC} ${LSSVM_LIBRARIES})
    TARGET_LINK_LIBRARIES(${TRAIN_EXEC} ${LSSVM_LIB_DEPENDS})
    TARGET_LINK_LIBRARIES(${PREDICT_EXEC} ${LSSVM_LIB_DEPENDS})
    INCLUDE_DIRECTORIES(${LSSVM_INCLUDE_DIRS})
    #MESSAGE (STATUS "Libraries: ${LSSVM_LIBRARIES}")
    #MESSAGE (STATUS "Depends: ${LSSVM_LIB_DEPENDS}")
    #MESSAGE (STATUS "Include: ${LSSVM_INCLUDE_DIRS}")
ENDIF(USE_LSSVMATLAS)


# Add our include files into our compiler's search path.
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

INSTALL_TARGETS(/bin ${TRAIN_EXEC} ${PREDICT_EXEC} ${TEST_EXEC})
