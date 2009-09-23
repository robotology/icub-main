SET(PROJECTNAME yarpIFlib)
PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP REQUIRED)

SET(folder_header yarpIFlib.hpp)
SET(folder_source yarpIFlib.cpp)

ADD_LIBRARY(${PROJECTNAME} ${folder_header} ${folder_source})




