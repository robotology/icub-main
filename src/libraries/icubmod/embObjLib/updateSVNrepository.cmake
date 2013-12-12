# Copyright: (C) 2013 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

find_package(Subversion)
if(NOT Subversion_FOUND)
    message(FATAL_ERROR "Cannot find Subversion")
endif()



set(eBcode_BASE_URL "https://svn.code.sf.net/p/robotcub/code/trunk/iCub/firmware/emBODY/eBcode/")
if(EXISTS ${CMAKE_SOURCE_DIR}/.svn)
  Subversion_WC_INFO(${CMAKE_CURRENT_SOURCE_DIR} ICUB)
  set(eBcode_REVISION ${ICUB_WC_REVISION})
  set(eBcode_REVISION_description "current iCub HEAD")
else()
  set(eBcode_REVISION 27643)
  set(eBcode_REVISION_description ${eBcode_REVISION})
endif()


if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/embObj)
    message(STATUS " Downloading embObjLib from repository")
    execute_process(COMMAND ${Subversion_SVN_EXECUTABLE} co -r${eBcode_REVISION} --depth empty ${eBcode_BASE_URL} --non-interactive embObj
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} OUTPUT_QUIET RESULT_VARIABLE res ERROR_VARIABLE err)

    # Check if subversion is working fine.
    # RESULT_VARIABLE will be 0 if everu=ythung was fine

    if(RESULT_VARIABLE)
        message(ERROR " CMake was not able to download library, maybe command line tool for subversion was not intalled? \n      Check wiki.icub.org for more info.")
        return()
    endif()

    execute_process(COMMAND ${Subversion_SVN_EXECUTABLE} up -r${eBcode_REVISION} --depth empty embobj --non-interactive
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/embObj OUTPUT_QUIET)
    execute_process(COMMAND ${Subversion_SVN_EXECUTABLE} up -r${eBcode_REVISION} --depth empty embobj/core --non-interactive
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/embObj OUTPUT_QUIET )
    execute_process(COMMAND ${Subversion_SVN_EXECUTABLE} up -r${eBcode_REVISION} --depth empty embobj/core/exec --non-interactive
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/embObj OUTPUT_QUIET)
    execute_process(COMMAND ${Subversion_SVN_EXECUTABLE} up -r${eBcode_REVISION} --depth empty embobj/plus --non-interactive
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/embObj OUTPUT_QUIET )
    execute_process(COMMAND ${Subversion_SVN_EXECUTABLE} up -r${eBcode_REVISION} --depth empty robotconfig --non-interactive
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/embObj OUTPUT_QUIET)

    execute_process(COMMAND ${Subversion_SVN_EXECUTABLE} up -r${eBcode_REVISION} embobj/core/core --non-interactive
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/embObj OUTPUT_QUIET)
    execute_process(COMMAND ${Subversion_SVN_EXECUTABLE} up -r${eBcode_REVISION} embobj/core/exec/yarp --non-interactive
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/embObj OUTPUT_QUIET)
    execute_process(COMMAND ${Subversion_SVN_EXECUTABLE} up -r${eBcode_REVISION} embobj/plus/comm-v1 --non-interactive
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/embObj OUTPUT_QUIET)
    execute_process(COMMAND ${Subversion_SVN_EXECUTABLE} up -r${eBcode_REVISION} embobj/plus/utils --non-interactive
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/embObj OUTPUT_QUIET)
    execute_process(COMMAND ${Subversion_SVN_EXECUTABLE} up -r${eBcode_REVISION} robotconfig/v1 --non-interactive
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/embObj OUTPUT_QUIET)
else()
    Subversion_WC_INFO(${CMAKE_CURRENT_SOURCE_DIR}/embObj eBcode)
    if(NOT eBcode_WC_REVISION EQUAL eBcode_REVISION)
        message(STATUS " Updating embObjLib from repository from version ${eBcode_WC_REVISION} to ${eBcode_REVISION_description}")
        execute_process(COMMAND ${Subversion_SVN_EXECUTABLE} up -r${eBcode_REVISION} --non-interactive
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/embObj OUTPUT_QUIET)
    endif()
endif()


