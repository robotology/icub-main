set(appname iCubCluster)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template ${CMAKE_CURRENT_SOURCE_DIR}/scripts/icub-cluster.py)
set(conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/cluster-config.xml)

if (__ICUB_INSTALL_APPLICATIONS__)
icub_app(${appname})
icub_app_install(${appname} FILES ${scripts} DESTINATION scripts FORCE_UPDATE)
icub_app_install(${appname} FILES ${conf} FORCE_UPDATE)
endif()

yarp_install(FILES ${conf} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname})

################ Copy/install cluster management scripts)

install(PROGRAMS scripts/icub-cluster.py COMPONENT Core DESTINATION bin)

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/cluster-scripts-cmd
      COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}
      COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/scripts/icub-cluster.py ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}
      COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/scripts/icub-cluster-server.sh ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}
      COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/scripts/icub-cluster-run.sh ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}
      COMMENT "Copying icub-cluster scripts to ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}")
add_custom_target(cluster-scripts ALL DEPENDS ${CMAKE_BINARY_DIR}/cluster-scripts-cmd)

