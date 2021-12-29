# Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(imuFilter
                    CATEGORY device
                    TYPE yarp::dev::ImuFilter
                    INCLUDE ImuFilter.h
                    DEPENDS "TARGET YARP::YARP_math"
                    DEFAULT ON)
if(ENABLE_imuFilter)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR})
  yarp_add_plugin(imuFilter ImuFilter.cpp
                            ImuFilter.h
                            PassThroughInertial.cpp
                            PassThroughInertial.h)
  target_link_libraries(imuFilter PRIVATE YARP::YARP_os
                                          YARP::YARP_sig
                                          YARP::YARP_dev
                                          YARP::YARP_math
                                          ctrlLib)
  yarp_install(TARGETS imuFilter
               EXPORT icub-targets
               COMPONENT Runtime
               LIBRARY DESTINATION  ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION  ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})
endif()
