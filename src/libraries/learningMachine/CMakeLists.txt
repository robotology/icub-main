SET(PROJECTNAME learningMachine)
PROJECT(${PROJECTNAME})

SET(LM_LIB ${PROJECTNAME})

SET(LM_HEADER
    include/iCub/learningMachine/DatasetRecorder.h
    include/iCub/learningMachine/DummyLearner.h
    include/iCub/learningMachine/FactoryT.h
    include/iCub/learningMachine/FixedRangeScaler.h
    include/iCub/learningMachine/IFixedSizeLearner.h
    include/iCub/learningMachine/IFixedSizeTransformer.h
    include/iCub/learningMachine/IMachineLearner.h
    include/iCub/learningMachine/IScaler.h
    include/iCub/learningMachine/ITransformer.h
    include/iCub/learningMachine/LSSVMLearner.h
    include/iCub/learningMachine/MachineCatalogue.h
    include/iCub/learningMachine/MachinePortable.h
    include/iCub/learningMachine/Normalizer.h
    include/iCub/learningMachine/PortableT.h
    include/iCub/learningMachine/RandomFeature.h
    include/iCub/learningMachine/RLSLearner.h
    include/iCub/learningMachine/ScaleTransformer.h
    include/iCub/learningMachine/Standardizer.h
    include/iCub/learningMachine/TransformerCatalogue.h
    include/iCub/learningMachine/TransformerPortable.h )

SET(LM_MACHINE_SRC
    src/DatasetRecorder.cpp
    src/DummyLearner.cpp
    src/IFixedSizeLearner.cpp
    src/LSSVMLearner.cpp
    src/RLSLearner.cpp )

SET(LM_TRANSFORMER_SRC
    src/FixedRangeScaler.cpp
    src/IFixedSizeTransformer.cpp
    src/IScaler.cpp
    src/Normalizer.cpp
    src/RandomFeature.cpp
    src/ScaleTransformer.cpp
    src/Standardizer.cpp )

# Declare groups of source and header files -- makes things pretty in MSVC.
SOURCE_GROUP("Source Files" FILES ${LM_MACHINE_SRC} ${LM_TRANSFORMER_SRC})
SOURCE_GROUP("Header Files" FILES ${LM_HEADER})

# add YARP include directories
INCLUDE_DIRECTORIES(${ACE_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

# Add our include files into our compiler's search path.
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

ADD_LIBRARY(${LM_LIB} ${LM_MACHINE_SRC} ${LM_TRANSFORMER_SRC} ${LM_HEADER})

TARGET_LINK_LIBRARIES(${LM_LIB} ${ACE_LIBRARIES} ${YARP_LIBRARIES})

icub_export_library(${LM_LIB} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                              EXTERNAL_INCLUDE_DIRS ${ACE_INCLUDE_DIRS}
                              DESTINATION include/iCub/learningMachine
                              FILES ${LM_HEADER})

