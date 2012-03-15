attrib -R ..\..\..\sys\env\api\eEcommon.h
attrib -R ..\..\..\sys\env\src\eEcommon.c
attrib -R ..\..\..\sys\env\api\eEmemorymap.h
copy /Y   *.c ..\..\..\sys\env\src
copy /Y   *.h ..\..\..\sys\env\api
attrib +R ..\..\..\sys\env\api\eEcommon.h
attrib +R ..\..\..\sys\env\src\eEcommon.c
attrib +R ..\..\..\sys\env\api\eEmemorymap.h

