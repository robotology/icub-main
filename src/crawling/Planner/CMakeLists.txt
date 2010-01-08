IF(WIN32)
cmake_minimum_required(VERSION 2.6)
ELSE(WIN32)
cmake_minimum_required(VERSION 2.4)
ENDIF(WIN32)

project(CrawlPlanner)
 
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ICUB)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/GLM)

add_definitions( /wd4250 )

SET(COMPILE_CRAWL_SUPERVISOR_GUI FALSE CACHE BOOL "Do you want to build the optional gui to display the potiential field when planning ?")

add_subdirectory(CrawlPlanner)
IF(COMPILE_SUPERVISOR_GUI)
	add_subdirectory(PlannerSupervisor)
ENDIF(COMPILE_SUPERVISOR_GUI)
 
 
