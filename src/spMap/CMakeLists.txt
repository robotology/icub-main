PROJECT(spMap)


SET(ICUB_DIR "${PROJECT_SOURCE_DIR}/../../" CACHE LOCATION "where is icub root?")

SET(INSTALL_DIR "${ICUB_DIR}")


INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

FILE(GLOB folder_header include/iCub/*.h)
FILE(GLOB folder_source src/*.cpp)


ADD_LIBRARY(spMap ${folder_source} ${folder_header})
INSTALL(TARGETS spMap DESTINATION "${INSTALL_DIR}/lib")



