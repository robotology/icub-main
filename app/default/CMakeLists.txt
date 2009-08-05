SET(APP_NAME default)

PROJECT($APP_NAME)

SET(appdir app/${APP_NAME})
SET(scripts "${ICUB_DIR}/${appdir}/scripts")
SET(conf "${ICUB_DIR}/${appdir}/conf")
SET(dapp "${ICUB_APP}/default")

INSTALL(CODE "MESSAGE(\"Installing ${APP_NAME}\")")

# some scripts
INSTALL(FILES 
  ${scripts}/cameras
  ${scripts}/checkNodes.sh
  ${scripts}/checkPorts.sh
  ${scripts}/cycle-fr.sh
  ${scripts}/cycle.sh
  ${scripts}/dragonfly
  ${scripts}/dragonfly-local
  ${scripts}/viewers
  ${scripts}/viewers-local
  ${scripts}/config.sh.template
  ${scripts}/config-nodes.sh.template
  ${scripts}/icub-cluster.sh
  ${scripts}/cluster-config.sh.template
  DESTINATION ${dapp}/scripts
  PERMISSIONS 
  OWNER_EXECUTE OWNER_READ OWNER_WRITE 
  GROUP_READ GROUP_WRITE
  WORLD_READ)

#INSTALL(FILES ${d}/config.sh DESTINATION ${appdir} RENAME config.ini)
#INSTALL(FILES ${d}/config.sh DESTINATION ${dest})
