
set(conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/cluster-config.xml)
yarp_install(FILES ${conf} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/yarpmanager)
