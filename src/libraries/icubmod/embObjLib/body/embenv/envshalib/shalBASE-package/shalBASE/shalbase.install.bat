
attrib -R ..\..\..\..\..\sys\env\api\shalBASE.h
copy /Y api\shalBASE.h ..\..\..\..\..\sys\env\api
attrib +R ..\..\..\..\..\sys\env\api\shalBASE.h

attrib -R ..\..\..\..\..\sys\env\lib\shalBASE.*.obj
copy /Y lib\shalBASE.*.obj ..\..\..\..\..\sys\env\lib
attrib +R ..\..\..\..\..\sys\env\lib\shalBASE.*.obj

attrib -R ...\.\..\..\..\sys\env\bin\shalbase.*.burnflash.bat
copy /Y bin\shalbase.*.burnflash.bat ..\..\..\..\..\sys\env\bin
attrib +R ..\..\..\..\..\sys\env\bin\shalbase.*.burnflash.bat

attrib -R ...\.\..\..\..\sys\env\bin\shalbase.*.burnflash.uvproj
copy /Y bin\shalbase.*.burnflash.uvproj ..\..\..\..\..\sys\env\bin
attrib +R ..\..\..\..\..\sys\env\bin\shalbase.*.burnflash.uvproj

attrib -R ...\.\..\..\..\sys\env\bin\shalbase.*.burnflash.uvopt
copy /Y bin\shalbase.*.burnflash.uvopt ..\..\..\..\..\sys\env\bin
attrib +R ..\..\..\..\..\sys\env\bin\shalbase.*.burnflash.uvopt

attrib -R ..\..\..\..\..\sys\env\bin\shalBASE.*.hex
copy /Y bin\shalBASE.*.hex ..\..\..\..\..\sys\env\bin
attrib +R ..\..\..\..\..\sys\env\bin\shalBASE.*.hex
