SET(PROJECTNAME iKinArmCtrl)
PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

FIND_PACKAGE(IKIN REQUIRED)
INCLUDE_DIRECTORIES(${IKIN_INCLUDE_DIRS})

IF(NOT IPOPT_FOUND)
   MESSAGE(FATAL_ERROR "IPOPT is strictly required!")
ENDIF(NOT IPOPT_FOUND)

SET(folder_header include/iCub/utils.hpp
                  include/iCub/solver.hpp
                  include/iCub/controller.hpp)

SET(folder_source src/utils.cpp
                  src/solver.cpp
                  src/controller.cpp
                  src/main.cpp)

SOURCE_GROUP("Header Files" FILES ${folder_header})
SOURCE_GROUP("Source Files" FILES ${folder_source})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

ADD_EXECUTABLE(${PROJECTNAME} ${folder_header} ${folder_source})
TARGET_LINK_LIBRARIES(${PROJECTNAME} ${IKIN_LIBRARIES})
INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)




