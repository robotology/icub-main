# Copyright: (C) 2025 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Jacopo Losi
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


yarp_prepare_plugin(fineCalibrationChecker
                    CATEGORY device
                    TYPE FineCalibrationChecker
                    INCLUDE src/FineCalibrationChecker.h
                    EXTRA_CONFIG
                    DEFAULT OFF)

if(ENABLE_fineCalibrationChecker)
  yarp_add_plugin(fineCalibrationChecker)

  target_sources(fineCalibrationChecker PRIVATE src/FineCalibrationChecker.cpp
                                              src/FineCalibrationChecker.h)

  target_link_libraries(fineCalibrationChecker PRIVATE YARP::YARP_os
                                                      YARP::YARP_dev
                                                      ICUB::iCubDev
                                                      ${OpenCV_LIBS})
                                                      
  list(APPEND YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS YARP_os
                                                      YARP_dev)

  yarp_install(TARGETS fineCalibrationChecker 
               EXPORT YARP_${YARP_PLUGIN_MASTER}
               COMPONENT ${YARP_PLUGIN_MASTER}
               LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

  set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

  set_property(TARGET fineCalibrationChecker PROPERTY FOLDER "Plugins/Device")
endif()
