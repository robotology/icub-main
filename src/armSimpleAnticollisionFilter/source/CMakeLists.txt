PROJECT(anticol)

LINK_DIRECTORIES("$ENV{HOME}/local/lib")

FIND_PACKAGE(YARP)

SET(ROBOOP_DIR "${CMAKE_SOURCE_DIR}/roboop")
FIND_PACKAGE(ROBOOP)

INCLUDE_DIRECTORIES("include" "$ENV{HOME}/local/include"  "." )

ADD_EXECUTABLE(anticolfilter anticollisionFilter.cpp iCub_kinematics_common.cpp)

