
# You'll need to tell CMake where to find YARP.
# From the command line, you can do:
#   cmake -DYARP_DIR:LOCATION=../.. .
# where ../.. is replaced with the path to which you built YARP.

FIND_PACKAGE(YARP)

PROJECT(audioAnalyser)

ADD_EXECUTABLE(simple_receiver simple_receiver.cpp)
ADD_EXECUTABLE(audioAnalyser main.cpp)