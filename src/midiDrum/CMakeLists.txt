PROJECT(midiDrum)

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP)

#FIND_PACKAGE(PPEVENTDEBUGGER)

#INCLUDE_DIRECTORIES(${GTKPLUS_INCLUDE_DIR})

FILE(GLOB folder_source *.cpp directmidi/*.cpp directmidi/dsutil/*.cpp)
FILE(GLOB folder_header *.h directmidi/*.h directmidi/dsutil/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

#LINK_LIBRARIES(
#    ${PPEVENTDEBUGGER_LIBRARIES}
#)

#INCLUDE_DIRECTORIES(
#    ${PROJECT_SOURCE_DIR}/include
#    ${PPEVENTDEBUGGER_INCLUDE_DIRS}
#)

ADD_EXECUTABLE(midiDrum ${folder_source} ${folder_header})

INSTALL_TARGETS(/bin midiDrum)

