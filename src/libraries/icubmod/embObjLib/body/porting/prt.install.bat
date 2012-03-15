attrib -R ..\..\sys\prt\*.h
attrib -R ..\..\sys\prt\*.c
copy /Y   *.c ..\..\sys\prt
copy /Y   *.h ..\..\sys\prt
attrib +R ..\..\sys\prt\*.h
attrib +R ..\..\sys\prt\*.c

