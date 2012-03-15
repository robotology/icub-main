
attrib -R ..\..\..\..\..\sys\env\api\shalIPAL.h
copy /Y api\shalIPAL.h ..\..\..\..\..\sys\env\api
attrib +R ..\..\..\..\..\sys\env\api\shalIPAL.h

attrib -R ..\..\..\..\..\sys\env\lib\shalIPAL.cm3.obj
copy /Y lib\shalIPAL.cm3.obj ..\..\..\..\..\sys\env\lib
attrib +R ..\..\..\..\..\sys\env\lib\shalIPAL.cm3.obj

attrib -R ..\..\..\..\..\sys\env\bin\shalIPALcm3.hex
copy /Y bin\shalIPALcm3.hex ..\..\..\..\..\sys\env\bin
attrib +R ..\..\..\..\..\sys\env\bin\shalIPALcm3.hex

attrib -R ...\.\..\..\..\sys\env\bin\shalipal.burnflash.*
copy /Y bin\shalipal.burnflash.* ..\..\..\..\..\sys\env\bin
attrib +R ..\..\..\..\..\sys\env\bin\shalipal.burnflash.*

