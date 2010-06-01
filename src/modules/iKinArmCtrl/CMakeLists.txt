SET(PROJECTNAME iKinArmCtrl)

PROJECT(${PROJECTNAME})

IF(NOT ICUB_USE_IPOPT)
   MESSAGE(FATAL_ERROR "${PROJECTNAME}: IPOPT is strictly required!")
ENDIF(NOT ICUB_USE_IPOPT)

SET(folder_header include/iCub/utils.hpp
                  include/iCub/solver.hpp
                  include/iCub/controller.hpp)

SET(folder_source src/utils.cpp
                  src/solver.cpp
                  src/controller.cpp
                  src/main.cpp)

SOURCE_GROUP("Header Files" FILES ${folder_header})
SOURCE_GROUP("Source Files" FILES ${folder_source})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${iKin_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_header} ${folder_source})

TARGET_LINK_LIBRARIES(${PROJECTNAME} iKin
                                     ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)




