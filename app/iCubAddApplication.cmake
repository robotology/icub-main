macro (iCubAddApplication app_name destination)

set(appdir ${app_name})
set(dapp "${destination}/${app_name}")

message(STATUS "Looking into ${CMAKE_CURRENT_SOURCE_DIR}/${appdir}/scripts")

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/${appdir}/scripts/*.template)
file(GLOB confs ${CMAKE_CURRENT_SOURCE_DIR}/${appdir}/conf/*.*)

message(STATUS "---> Found ${scripts}")
message(STATUS ${command1})

add_custom_command(OUTPUT copy
					COMMAND ${CMAKE_COMMAND} -E make_directory ${dapp}/scripts
					COMMAND ${CMAKE_COMMAND} -E make_directory ${dapp}/conf
					COMMENT "Creating directories ${dapp}/scripts and conf"
					APPEND)

foreach(s ${scripts})
set(command2 ${CMAKE_COMMAND} -E copy ${s} ${dapp}/scripts)
add_custom_command(OUTPUT copy
					COMMAND ${command2}
					COMMENT "Copy to ${s} to ${dapp}/scripts"
					APPEND)		
endforeach(s ${scripts})					

foreach(c ${confs})
set(command2 ${CMAKE_COMMAND} -E copy ${c} ${dapp}/conf)
add_custom_command(OUTPUT copy
					COMMAND ${command2}
					COMMENT "Copy to ${c} to ${dapp}/conf"
					APPEND)		
endforeach(c ${confs})	

endmacro()
