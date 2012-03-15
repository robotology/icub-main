
attrib -R ..\..\..\..\..\sys\env\api\shalHAL.h
copy /Y api\shalHAL.h ..\..\..\..\..\sys\env\api
attrib +R ..\..\..\..\..\sys\env\api\shalHAL.h

attrib -R ..\..\..\..\..\sys\env\lib\shalHAL.mcbstm32c.obj
copy /Y lib\shalHAL.mcbstm32c.obj ..\..\..\..\..\sys\env\lib
attrib +R ..\..\..\..\..\sys\env\lib\shalHAL.mcbstm32c.obj

attrib -R ..\..\..\..\..\sys\env\bin\shalHALmcbstm32c.hex
copy /Y bin\shalHALmcbstm32c.hex ..\..\..\..\..\sys\env\bin
attrib +R ..\..\..\..\..\sys\env\bin\shalHALmcbstm32c.hex

attrib -R ...\.\..\..\..\sys\env\bin\shalhal.burnflash.*
copy /Y bin\shalhal.burnflash.* ..\..\..\..\..\sys\env\bin
attrib +R ..\..\..\..\..\sys\env\bin\shalhal.burnflash.*

