
attrib -R ..\..\..\..\..\sys\env\api\shalFSAL.h
copy /Y api\shalFSAL.h ..\..\..\..\..\sys\env\api
attrib +R ..\..\..\..\..\sys\env\api\shalFSAL.h

attrib -R ..\..\..\..\..\sys\env\lib\shalFSALcm3.obj
copy /Y lib\shalFSAL.cm3.obj ..\..\..\..\..\sys\env\lib
attrib +R ..\..\..\..\..\sys\env\lib\shalFSAL.cm3.obj

attrib -R ..\..\..\..\..\sys\env\bin\shalFSALcm3.hex
copy /Y bin\shalFSALcm3.hex ..\..\..\..\..\sys\env\bin
attrib +R ..\..\..\..\..\sys\env\bin\shalFSALcm3.hex

attrib -R ...\.\..\..\..\sys\env\bin\shalfsal.burnflash.*
copy /Y bin\shalfsal.burnflash.* ..\..\..\..\..\sys\env\bin
attrib +R ..\..\..\..\..\sys\env\bin\shalfsal.burnflash.*

