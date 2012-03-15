

attrib -R ..\..\..\..\..\sys\env\api\shalOSAL.h
copy /Y api\shalOSAL.h ..\..\..\..\..\sys\env\api
attrib +R ..\..\..\..\..\sys\env\api\shalOSAL.h

attrib -R ..\..\..\..\..\sys\env\lib\shalOSAL.cm3.obj
copy /Y lib\shalOSAL.cm3.obj ..\..\..\..\..\sys\env\lib
attrib +R ..\..\..\..\..\sys\env\lib\shalOSAL.cm3.obj

attrib -R ..\..\..\..\..\sys\env\bin\shalOSALcm3.hex
copy /Y bin\shalOSALcm3.hex ..\..\..\..\..\sys\env\bin
attrib +R ..\..\..\..\..\sys\env\bin\shalOSALcm3.hex

attrib -R ...\.\..\..\..\sys\env\bin\shalosal.burnflash.*
copy /Y bin\shalosal.burnflash.* ..\..\..\..\..\sys\env\bin
attrib +R ..\..\..\..\..\sys\env\bin\shalosal.burnflash.*

