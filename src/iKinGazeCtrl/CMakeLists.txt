SET(PROJECTNAME iKinGazeCtrl)
PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)
FIND_PACKAGE(IKIN REQUIRED)

IF(NOT IPOPT_FOUND)
   MESSAGE(FATAL_ERROR "IPOPT is strictly required!")
ENDIF(NOT IPOPT_FOUND)

SET(folder_header include/iCub/gazeNlp.hpp
                  include/iCub/utils.hpp
                  include/iCub/solver.hpp
                  include/iCub/controller.hpp
                  include/iCub/localizer.hpp)

SET(folder_source src/gazeNlp.cpp
                  src/utils.cpp
                  src/solver.cpp
                  src/controller.cpp
                  src/localizer.cpp
                  src/main.cpp)

SOURCE_GROUP("Header Files" FILES ${folder_header})
SOURCE_GROUP("Source Files" FILES ${folder_source})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include ${IKIN_INCLUDE_DIRS})
ADD_EXECUTABLE(${PROJECTNAME} ${folder_header} ${folder_source})
TARGET_LINK_LIBRARIES(${PROJECTNAME} ${IKIN_LIBRARIES})

IF(MSVC)
   SET_TARGET_PROPERTIES(${PROJECTNAME} PROPERTIES COMPILE_FLAGS /wd4996)
ENDIF(MSVC)

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)




