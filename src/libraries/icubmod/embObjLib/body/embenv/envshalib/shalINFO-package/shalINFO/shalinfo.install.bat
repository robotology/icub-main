
attrib -R ..\..\..\..\..\sys\env\api\shalINFO.h
copy /Y api\shalINFO.h ..\..\..\..\..\sys\env\api
attrib +R ..\..\..\..\..\sys\env\api\shalINFO.h

attrib -R ..\..\..\..\..\sys\env\lib\shalINFO.*.obj
copy /Y lib\shalINFO.*.obj ..\..\..\..\..\sys\env\lib
attrib +R ..\..\..\..\..\sys\env\lib\shalINFO.*.obj

attrib -R ...\.\..\..\..\sys\env\bin\shalinfo.*.burnflash.bat
copy /Y bin\shalinfo.*.burnflash.bat ..\..\..\..\..\sys\env\bin
attrib +R ..\..\..\..\..\sys\env\bin\shalinfo.*.burnflash.bat

attrib -R ...\.\..\..\..\sys\env\bin\shalinfo.*.burnflash.uvproj
copy /Y bin\shalinfo.*.burnflash.uvproj ..\..\..\..\..\sys\env\bin
attrib +R ..\..\..\..\..\sys\env\bin\shalinfo.*.burnflash.uvproj

attrib -R ...\.\..\..\..\sys\env\bin\shalinfo.*.burnflash.uvopt
copy /Y bin\shalinfo.*.burnflash.uvopt ..\..\..\..\..\sys\env\bin
attrib +R ..\..\..\..\..\sys\env\bin\shalinfo.*.burnflash.uvopt

attrib -R ..\..\..\..\..\sys\env\bin\shalINFO.*.hex
copy /Y bin\shalINFO.*.hex ..\..\..\..\..\sys\env\bin
attrib +R ..\..\..\..\..\sys\env\bin\shalINFO.*.hex

