# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(pcan TYPE yarp::dev::PlxCan
    CATEGORY device
    INCLUDE PlxCan.h )

if (NOT SKIP_pcan)
  FIND_PACKAGE(PLXCANAPI)

  IF (NOT PLXCANAPI_FOUND)
    MESSAGE("pcan: cannot find plxcanapi, turn off device")
  ELSE (NOT PLXCANAPI_FOUND)
        INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}
                            ${PLXCANAPI_INC_DIRS})

        #plx-api related define
        ADD_DEFINITIONS(-DPLX_LITTLE_ENDIAN -DPLX_LINUX -DGCC)
        yarp_add_plugin(pcan PlxCan.cpp PlxCan.h)

        TARGET_LINK_LIBRARIES(pcan ${PLXCANAPI_LIB} ${YARP_LIBRARIES})

        icub_export_plugin(pcan)

  yarp_install(TARGETS pcan
               COMPONENT Runtime
               LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

  ENDIF (NOT PLXCANAPI_FOUND)

endif (NOT SKIP_pcan)

