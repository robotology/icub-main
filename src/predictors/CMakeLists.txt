PROJECT(predictors)

FIND_PACKAGE(ICUB QUIET)
IF (NOT ICUB_FOUND)
   MESSAGE (STATUS "ICUB repository not found; assuming distributed build.")
   MESSAGE (STATUS "If this is not what you want, set ICUB_DIR variable.")
ENDIF (NOT ICUB_FOUND)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

FILE(GLOB folder_header include/iCub/*.h)
FILE(GLOB folder_source src/*.cpp)

ADD_LIBRARY(predictors ${folder_source} ${folder_header})
INSTALL(TARGETS predictors DESTINATION lib)
INSTALL(FILES ${folder_header} DESTINATION include/iCub)




