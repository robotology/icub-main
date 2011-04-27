set(appname robotScripting)

icub_app(${appname})

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml.template)
file(GLOB hannover_left ${CMAKE_CURRENT_SOURCE_DIR}/hannover/left/*)
file(GLOB hannover_right ${CMAKE_CURRENT_SOURCE_DIR}/hannover/right/*)
file(GLOB frizzi_collegamento ${CMAKE_CURRENT_SOURCE_DIR}/frizzi/collegamento/*)

icub_app_install(${appname} 
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/scripting.py)	

icub_app_install(${appname}
  FILES ${scripts}
  DESTINATION scripts)

# hannover demo scripting
icub_app_install(${appname}
  FILES ${hannover_left}
  DESTINATION hannover/left)

icub_app_install(${appname}
  FILES ${hannover_right}
  DESTINATION hannover/right)

# frizzi (quirinale) demo scripting
icub_app_install(${appname}
  FILES ${frizzi_collegamento}
  DESTINATION frizzi/sequence)

icub_app_install(${appname}
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/frizzi/icub-speak
  DESTINATION frizzi)



