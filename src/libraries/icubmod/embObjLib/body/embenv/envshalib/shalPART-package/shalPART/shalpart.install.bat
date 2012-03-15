
attrib -R ..\..\..\..\..\sys\env\api\shalPART.h
copy /Y api\shalPART.h ..\..\..\..\..\sys\env\api
attrib +R ..\..\..\..\..\sys\env\api\shalPART.h

attrib -R ..\..\..\..\..\sys\env\lib\shalPART.*.obj
copy /Y lib\shalPART.*.obj ..\..\..\..\..\sys\env\lib
attrib +R ..\..\..\..\..\sys\env\lib\shalPART.*.obj

attrib -R ...\.\..\..\..\sys\env\bin\shalpart.*.burnflash.bat
copy /Y bin\shalpart.*.burnflash.bat ..\..\..\..\..\sys\env\bin
attrib +R ..\..\..\..\..\sys\env\bin\shalpart.*.burnflash.bat

attrib -R ...\.\..\..\..\sys\env\bin\shalpart.*.burnflash.uvproj
copy /Y bin\shalpart.*.burnflash.uvproj ..\..\..\..\..\sys\env\bin
attrib +R ..\..\..\..\..\sys\env\bin\shalpart.*.burnflash.uvproj

attrib -R ...\.\..\..\..\sys\env\bin\shalpart.*.burnflash.uvopt
copy /Y bin\shalpart.*.burnflash.uvopt ..\..\..\..\..\sys\env\bin
attrib +R ..\..\..\..\..\sys\env\bin\shalpart.*.burnflash.uvopt

attrib -R ..\..\..\..\..\sys\env\bin\shalPART.*.hex
copy /Y bin\shalPART.*.hex ..\..\..\..\..\sys\env\bin
attrib +R ..\..\..\..\..\sys\env\bin\shalPART.*.hex


