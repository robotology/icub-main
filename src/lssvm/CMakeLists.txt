cmake_minimum_required(VERSION 2.6)

# This is the liblssvm library which provides a Least Squares Support Vector
# Machine.
SET(PROJECTNAME LSSVM)
PROJECT(${PROJECTNAME})

# define executables
SET(LSSVM_TRAIN_EXEC lssvm-train)
SET(LSSVM_PREDICT_EXEC lssvm-predict)
SET(LSSVM_SCALE_EXEC lssvm-scale)
SET(KTA_EXEC kta)

# set compilation flags
SET(CMAKE_CXX_FLAGS "-O3")

FIND_PACKAGE(ICUB QUIET)
IF (NOT ICUB_FOUND)
   MESSAGE (STATUS "ICUB repository not found; assuming distributed build.")
   MESSAGE (STATUS "If this is not what you want, set ICUB_DIR variable.")
ENDIF (NOT ICUB_FOUND)

# Find .cpp and .h files automatically.  This is a bit lazy,
# and in principle it would be better to list these files manually.
# 
# Basically the Problem class could be removed from the library and compiled
# just into the executables that need it. This can be done in a later stage.
FILE(GLOB folder_source src/lib/*.cpp)
FILE(GLOB folder_header include/iCub/*.h)

# Declare groups of source and header files -- makes things pretty in MSVC.
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Add our include files into our compiler's search path.
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

# Create everything needed to build our executable.
ADD_EXECUTABLE(${LSSVM_TRAIN_EXEC} src/bin/lssvm-train.cpp)
ADD_EXECUTABLE(${LSSVM_PREDICT_EXEC} src/bin/lssvm-predict.cpp)
ADD_EXECUTABLE(${LSSVM_SCALE_EXEC} src/bin/lssvm-scale.cpp)
ADD_EXECUTABLE(${KTA_EXEC} src/bin/kta.cpp)

# Create everything needed to build our executable.
ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})

##########################
#
# this could use a conditional statement whether we want a threaded variant or not
#
##########################

# look for the Lapack/Atlas library
FIND_LIBRARY(LAPACKATLAS_LIB NAMES lapack_atlas REQUIRED)
IF (LAPACKATLAS_LIB)
  TARGET_LINK_LIBRARIES (${PROJECTNAME} ${LAPACKATLAS_LIB})
ELSE (LAPACKATLAS_LIB)
  MESSAGE(FATAL_ERROR "Could not find lapack/atlas library.")
ENDIF (LAPACKATLAS_LIB)

# look for the Lapack library
FIND_LIBRARY(LAPACK_LIB NAMES lapack REQUIRED)
IF (LAPACK_LIB)
  TARGET_LINK_LIBRARIES (${PROJECTNAME} ${LAPACK_LIB})
ELSE (LAPACK_LIB)
  MESSAGE(FATAL_ERROR "Could not find lapack library.")
ENDIF (LAPACK_LIB)

# look for the cblas library (threaded is preferred)
FIND_LIBRARY(CBLAS_LIB NAMES ptcblas cblas REQUIRED)
IF (CBLAS_LIB)
  TARGET_LINK_LIBRARIES (${PROJECTNAME} ${CBLAS_LIB})
ELSE (CBLAS_LIB)
  MESSAGE(FATAL_ERROR "Could not find cblas library.")
ENDIF (CBLAS_LIB)

#look for f77blas library
FIND_LIBRARY(F77BLAS_LIB NAMES ptf77blas f77blas REQUIRED)
IF (F77BLAS_LIB)
  TARGET_LINK_LIBRARIES (${PROJECTNAME} ${F77BLAS_LIB})
ELSE (F77BLAS_LIB)
  MESSAGE(FATAL_ERROR "Could not find f77cblas library.")
ENDIF (F77BLAS_LIB)


# look for the Atlas library
FIND_LIBRARY(ATLAS_LIB NAMES atlas REQUIRED)
IF (ATLAS_LIB)
  TARGET_LINK_LIBRARIES (${PROJECTNAME} ${ATLAS_LIB})
ELSE (ATLAS_LIB)
  MESSAGE(FATAL_ERROR "Could not find atlas library.")
ENDIF (ATLAS_LIB)

# look for the Pthread library
FIND_LIBRARY(PTHREAD_LIB NAMES pthread REQUIRED)
IF (PTHREAD_LIB)
  TARGET_LINK_LIBRARIES (${PROJECTNAME} ${PTHREAD_LIB})
ELSE (PTHREAD_LIB)
  MESSAGE(FATAL_ERROR "Could not find pthread library.")
ENDIF (PTHREAD_LIB)

TARGET_LINK_LIBRARIES(${LSSVM_TRAIN_EXEC} ${PROJECTNAME})
TARGET_LINK_LIBRARIES(${LSSVM_PREDICT_EXEC} ${PROJECTNAME})
TARGET_LINK_LIBRARIES(${LSSVM_SCALE_EXEC} ${PROJECTNAME})
TARGET_LINK_LIBRARIES(${KTA_EXEC} ${PROJECTNAME})

# export library dependencies for use in other projects
INCLUDE(CMakeExportBuildSettings)
  
CMAKE_EXPORT_BUILD_SETTINGS(${PROJECT_BINARY_DIR}/LSSVMBuildSettings.cmake)
EXPORT_LIBRARY_DEPENDENCIES(${PROJECT_BINARY_DIR}/LSSVMLibraryDepends.cmake)
