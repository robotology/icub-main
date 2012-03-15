
-----------------------------------------------------------------------------------------------------------------------
- iSharINFO
-----------------------------------------------------------------------------------------------------------------------


The iSharINFO is a shared module to be used in the STM32 micro which offers common access to data info.



-----------------------------------------------------------------------------------------------------------------------
- HOW TO INSTALL
-----------------------------------------------------------------------------------------------------------------------

Compile using project ishared-info.
- copy file iSharINFO.obj into C:\iEmb\shared\isharinfo\lib
- copy file iSharIPC.h into C:\iEmb\shared\isharinfo\api
- burn flash of the STM32 with iSharINFO.hex. Use project ishared-info or a yet-to-be-developed installer
  the ishared-ipc puts the iSharINFO.hex into FLASH partition [6K, 10K) at address flashbase+6K and of size 4KB.  
  CAVEAT: the iSharIPC.hex uses dedicated RAM at address ?? for a total of ??.


-----------------------------------------------------------------------------------------------------------------------
- HOW TO USE
-----------------------------------------------------------------------------------------------------------------------

Include the following into your project:
- file with symbols: iSharINFO.obj
- api file: iSharINFO.h
- tell the linker to exclude: FLASH partition [8K, 10K) and RAM partition ???.
- in use of the iSharINFO, inside your code ALWAYS: verify the release version in the iSharINFO.h vs what obtained by 
  function sharinfo_version_get(&ver).




-----------------------------------------------------------------------------------------------------------------------
- HOW IT WAS DEVELOPED
-----------------------------------------------------------------------------------------------------------------------

By hand. 



-----------------------------------------------------------------------------------------------------------------------
- HOW TO UPGRADE IT
-----------------------------------------------------------------------------------------------------------------------

No need if it works as everything is self contained.  However, .... if CMSIS is upgraded you can also upgrade iSharINFO.



