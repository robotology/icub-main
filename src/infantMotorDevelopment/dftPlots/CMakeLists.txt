
# This is just an example library, that does not do anything much.  It
# is intended to be a quick way to understand current good practice
# for making iCub libraries.  This hasn't really stabilized yet, so if
# you see something that is out of date, please feel free to fix it

SET(PROJECTNAME borisPanel)

PROJECT(${PROJECTNAME})

FILE(GLOB folder_source src/plotsMain.cpp)

# Declare groups of source and header files -- makes things pretty in MSVC.
#SOURCE_GROUP("Source Files" FILES ${folder_source})
#SOURCE_GROUP("Header Files" FILES ${folder_header})

# Add our include files into our compiler's search path.
#INCLUDE_DIRECTORIES(${OPENCV_INCLUDE_DIR})
#INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

LINK_LIBRARIES(${OPENCV_LIBRARIES})

# Create everything needed to build our executable.
ADD_EXECUTABLE(${PROJECTNAME} ${folder_source})


