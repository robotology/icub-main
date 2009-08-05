PROJECT(predictors)


SET(ICUB_DIR "${PROJECT_SOURCE_DIR}/../../" CACHE LOCATION "where is icub root?")

SET(INSTALL_DIR "${ICUB_DIR}")


INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

FILE(GLOB folder_header include/iCub/*.h)
FILE(GLOB folder_source src/*.cpp)


ADD_LIBRARY(predictors ${folder_source} ${folder_header})
INSTALL(TARGETS predictors DESTINATION "${INSTALL_DIR}/lib")



