

PROJECT(arm_ctl)
INCLUDE (/usr/share/cmake-2.4/Modules/FindOpenGL.cmake)

SET(YARP_DIR_HARD_CODED "/opt/yarp-2.2.1")
#FIND_PATH(with_yarp YARPConfig.cmake YARP_DIR)
FIND_PATH(with_yarp YARPConfig.cmake YARP_DIR_HARD_CODED)
IF(NOT with_yarp-NOTFOUND)
 
FIND_PACKAGE(YARP)

ENDIF(NOT with_yarp-NOTFOUND)

FIND_PACKAGE(OpenGL)

INCLUDE_DIRECTORIES(include)


IF (WIN32 AND NOT CYGWIN)
  ADD_DEFINITIONS(-DWIN32 -D_WINDOWS)
ENDIF (WIN32 AND NOT CYGWIN)

IF(WIN32 AND CYGWIN)
  ADD_DEFINITIONS(-DCYGWIN)
ENDIF(WIN32 AND CYGWIN)


SET(CMAKE_CXX_FLAGS "-DTRUE=1 -DFALSE=0 -DYARP_COMM -O3 -Wall -pg")
SET(CMAKE_VERBOSE_MAKEFILE ON)
SET(CMAKE_EXE_LINKER_FLAGS "-L/usr/lib -L/lib -lGLU -lGL -lglut -lstdc++ -lm")
#SET(CMAKE_EXE_LINKER_FLAGS "-L/usr/lib -lstdc++ -lm")




INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})


SET (GEN_REACHING_SOURCES
    src/Vector.cpp
    src/Matrix.cpp
    src/mathlib.cpp
    src/TreeParser.cpp
    src/KinematicChain.cpp
    src/Rotation.cpp
    src/Translation.cpp
    src/RigidTransfo.cpp
    src/KChainBodySchema.cpp
    src/KChainOrientationBodySchema.cpp 
    src/ReachingGen.cpp
)


SET (REACHING_MODULE_SOURCES
    ${GEN_REACHING_SOURCES}
    src/KinematicTree.cpp
    src/SpecializedPort.cpp
    src/ReachingModuleThread.cpp
)


SET(BODY_SCHEMA_LEARNING_SOURCES
    src/Vector.cpp
    src/Matrix.cpp
    src/mathlib.cpp
    src/TreeParser.cpp
    src/KinematicChain.cpp
    src/KinematicTree.cpp
    src/Rotation.cpp
    src/Translation.cpp
    src/RigidTransfo.cpp
#    src/KChainBodySchema.cpp	
    src/SpecializedPort.cpp
    src/BodySchemaLearning.cpp
)

SET (PERCEPTION_SIMULATION_SOURCES
    src/Vector.cpp
    src/Matrix.cpp
    src/mathlib.cpp
    src/TreeParser.cpp
    src/KinematicChain.cpp
    src/Rotation.cpp
    src/Translation.cpp
    src/RigidTransfo.cpp
    src/SpecializedPort.cpp
    src/KinematicTree.cpp
    src/PerceptionSimulationModule.cpp
)

SET (NEW_SIMULATOR_SOURCES
	src/mathlib.cpp 
	src/Geometry.cpp 
	src/TreeParser.cpp	
	src/Timer.cpp
	src/GLTools.cpp 
	src/GL2DPlot.cpp 
	src/GLConsole.cpp 
	src/GLSnapshot.cpp 
	src/GLSubWindow.cpp 
	src/GLColorPlot.cpp 
	src/GLCamera.cpp 
	src/Console.cpp
	src/gl2ps.c 
#	src/SolidTree.cpp 
	src/GLUTApplication.cpp
	src/Rotation.cpp
	src/RigidTransfo.cpp
	src/Translation.cpp
	src/KinematicChain.cpp
	src/KinematicTree.cpp
	src/EmbodiedArticulatedTree.cpp
    	src/Shape.cpp
	src/SpecializedPort.cpp
	src/EmbodiedKinematicChain.cpp
	src/KinematicSimulator.cpp
)

SET (HEAD_TO_BODY_CENTERED_SOURCES
      src/Vector.cpp		
    src/Matrix.cpp		
    src/mathlib.cpp		
    src/TreeParser.cpp		
    src/KinematicChain.cpp	
    src/Rotation.cpp		
    src/Translation.cpp		
    src/RigidTransfo.cpp	
    src/KChainBodySchema.cpp
#    src/KChainOrientationBodySchema.cpp
    src/KinematicTree.cpp
    src/SpecializedPort.cpp
    src/HeadToBodyCenteredRef.cpp
)

SET (BODY_SCHEMA_LOGGER_SOURCES
    src/TreeParser.cpp	
    src/mathlib.cpp
    src/KinematicChain.cpp
    src/KinematicTree.cpp		
    src/Rotation.cpp		
    src/Translation.cpp		
    src/RigidTransfo.cpp	
    src/BodySchemaLogger.cpp
    src/SpecializedPort.cpp
)

IF (NOT WIN32)
   LINK_LIBRARIES(ncurses)
ENDIF (NOT WIN32)


SET(CMAKE_CXX_FLAGS "-O3 -Wall")## -g -pg

ADD_DEFINITIONS(-DBODY_SCHEMA -DTRUE=1 -DFALSE=0 -DGOAL_DIRECTED_MAIN -DYARP_ROBOT -DNEW_SERVER_PROCESS) 


ADD_EXECUTABLE(reaching_module ${REACHING_MODULE_SOURCES} )  

ADD_EXECUTABLE(body_schema_module ${BODY_SCHEMA_LEARNING_SOURCES}) 
 
ADD_EXECUTABLE(perception_module ${PERCEPTION_SIMULATION_SOURCES})  

ADD_EXECUTABLE(simulator ${NEW_SIMULATOR_SOURCES})

ADD_EXECUTABLE(head_transfo_module ${HEAD_TO_BODY_CENTERED_SOURCES})

ADD_EXECUTABLE(logger ${BODY_SCHEMA_LOGGER_SOURCES})
