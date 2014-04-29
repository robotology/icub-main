# Copyright: (C) 2013 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


if(icub_firmware_shared_embobj_FOUND)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/embObj)
        message("Deleting old embObj folder")
        file(REMOVE_RECURSE ${CMAKE_CURRENT_SOURCE_DIR}/embObj)
    endif()
    return()
endif()


message(WARNING "
  embObj library can now be found in the icub-firmware-shared package that
  can be downloaded from here:
  .
       https://github.com/robotology/icub-firmware-shared
  .
  In order not to break existing build, it will now be downloaded and built
  automatically, but this will be disabled in the future, therefore you
  should install it as soon as possible.
")
return()

find_package(Subversion)
if(NOT Subversion_FOUND)
    if(WIN32)
        message(FATAL_ERROR "\nCannot find Subversion executable, maybe command line tool for subversion was not installed?  Check wiki.icub.org for more info.\n")
    else(WIN32)
        message(FATAL_ERROR "Cannot find Subversion, you can do 'sudo apt-get install subversion' to install it.")
    endif(WIN32)
endif()


set(eBcode_BASE_OLD_URL "https://svn.code.sf.net/p/robotcub/code/trunk/iCub/firmware/emBODY/eBcode")
set(eBcode_BASE_URL "https://github.com/robotology/icub-firmware-shared/trunk/eth")

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/embObj)
    Subversion_WC_INFO(${CMAKE_CURRENT_SOURCE_DIR}/embObj embObj)
    if("${embObj_WC_URL}" STREQUAL "${eBcode_BASE_OLD_URL}")
        file(REMOVE_RECURSE ${CMAKE_CURRENT_SOURCE_DIR}/embObj)
    endif()
endif()
#  Subversion_WC_INFO(${CMAKE_CURRENT_SOURCE_DIR} ICUB)

set(eBcode_REVISION HEAD)
set(eBcode_REVISION_description ${eBcode_REVISION})


if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/embObj)
    message(STATUS " Downloading embObjLib from repository")
    execute_process(COMMAND ${Subversion_SVN_EXECUTABLE} co -r${eBcode_REVISION} --depth empty ${eBcode_BASE_URL} --non-interactive embObj
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} OUTPUT_QUIET)

    execute_process(COMMAND ${Subversion_SVN_EXECUTABLE} up -r${eBcode_REVISION} --depth empty embobj --non-interactive
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/embObj OUTPUT_QUIET)
    execute_process(COMMAND ${Subversion_SVN_EXECUTABLE} up -r${eBcode_REVISION} --depth empty embobj/core --non-interactive
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/embObj OUTPUT_QUIET )
    execute_process(COMMAND ${Subversion_SVN_EXECUTABLE} up -r${eBcode_REVISION} --depth empty embobj/core/exec --non-interactive
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/embObj OUTPUT_QUIET)
    execute_process(COMMAND ${Subversion_SVN_EXECUTABLE} up -r${eBcode_REVISION} --depth empty embobj/plus --non-interactive
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/embObj OUTPUT_QUIET )

    execute_process(COMMAND ${Subversion_SVN_EXECUTABLE} up -r${eBcode_REVISION}  --depth empty embobj/plus/comm-v1 --non-interactive
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/embObj OUTPUT_QUIET)

    execute_process(COMMAND ${Subversion_SVN_EXECUTABLE} up -r${eBcode_REVISION} --depth empty robotconfig --non-interactive
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/embObj OUTPUT_QUIET)

    execute_process(COMMAND ${Subversion_SVN_EXECUTABLE} up -r${eBcode_REVISION} embobj/core/core --non-interactive
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/embObj OUTPUT_QUIET)
    execute_process(COMMAND ${Subversion_SVN_EXECUTABLE} up -r${eBcode_REVISION} embobj/core/exec/yarp --non-interactive
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/embObj OUTPUT_QUIET)

    execute_process(COMMAND ${Subversion_SVN_EXECUTABLE} up -r${eBcode_REVISION} embobj/plus/comm-v1/opcprot --non-interactive
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/embObj OUTPUT_QUIET)

    execute_process(COMMAND ${Subversion_SVN_EXECUTABLE} up -r${eBcode_REVISION} embobj/plus/comm-v2 --non-interactive
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


