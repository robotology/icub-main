set(appname iCubCluster)

set(conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/cluster-config.xml)
yarp_install(FILES ${conf} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname})

################ Copy/install cluster management scripts)

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/cluster-scripts-cmd
      COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}
      COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/scripts/icub-cluster.py ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}
      COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/scripts/icub-cluster-log.py ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}
      COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/scripts/icub-cluster-server.sh ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}
      COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/scripts/icub-cluster-run.sh ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}
      COMMENT "Copying icub-cluster scripts to ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}")
add_custom_target(cluster-scripts ALL DEPENDS ${CMAKE_BINARY_DIR}/cluster-scripts-cmd)

install(PROGRAMS scripts/icub-cluster.py
                 scripts/icub-cluster-log.py
                 scripts/icub-cluster-server.sh
                 scripts/icub-cluster-run.sh
        COMPONENT Core DESTINATION bin)
        