IF(WIN32)
cmake_minimum_required(VERSION 2.6)
ELSE(WIN32)
cmake_minimum_required(VERSION 2.4)
ENDIF(WIN32)

PROJECT(plannerSupervisor)


FIND_PACKAGE(GtkMM REQUIRED)

ADD_DEFINITIONS(${GTKMM_C_FLAGS})
LINK_LIBRARIES(${GTKMM_LINK_FLAGS})
INCLUDE_DIRECTORIES(${GTKMM_INCLUDE_DIR})
# SET(LIBXML_INCLUDE_DIR_1 $ENV{GTKMM_BASEPATH}/lib/libxml++-2.6/include)
# SET(LIBXML_INCLUDE_DIR_2 $ENV{GTKMM_BASEPATH}/include/libxml++-2.6)

# include_directories(${LIBXML_INCLUDE_DIR_1} ${LIBXML_INCLUDE_DIR_2})
 
FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h )
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

ADD_EXECUTABLE(plannerSupervisor ${folder_source} ${folder_header})
TARGET_LINK_LIBRARIES(plannerSupervisor ${GTKMM_LINK_FLAGS})
# target_link_libraries(PlannerSupervisor ${GTKMM_LIBRARIES} xml++-vc90-2_6.lib)

