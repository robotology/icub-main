mkdir rep
c:\keil\uv4\uv4 -f shalpart.mcbstm32c.burnflash.uvproj -orep/shalpart.mcbstm32c.report.txt
del -Y *.bak
del -Y *.dep
del -Y *.plg
del -Y *.uvgui*
