
-----------------------------------------------------------------------------------------------------------------------
- iSharIPC
-----------------------------------------------------------------------------------------------------------------------


The iSharIPC is a shared module to be used in the STM32 micro which offers Inter iProcess Communication.



-----------------------------------------------------------------------------------------------------------------------
- HOW TO INSTALL
-----------------------------------------------------------------------------------------------------------------------

Compile using project ishared-ipc.
- copy file iSharIPC.obj into C:\iEmb\shared\isharipc\lib
- copy file iSharIPC.h into C:\iEmb\shared\isharipc\api
- burn flash of the STM32 with iSharIPC.hex. Use project ishared-ipc or a yet-to-be-developed installer
  the ishared-ipc puts the iSharIPC.hex into FLASH partition [4K, 6K) at address flashbase+4K and of size 2KB.  
  CAVEAT: the iSharIPC.hex uses dedicated RAM [0, 16) at address rambase for a total of 16B.


-----------------------------------------------------------------------------------------------------------------------
- HOW TO USE
-----------------------------------------------------------------------------------------------------------------------

Include the following into your project:
- file with symbols: iSharIPC.obj
- api file: iSharIPC.h
- tell the linker to exclude: FLASH partition [4K, 6K) and RAM partition [0, 16).
- in use of the iSharIPC, inside your code ALWAYS: verify the release version in the iSharIPC.h vs what obtained by 
  function sharipc_version_get(&ver).




-----------------------------------------------------------------------------------------------------------------------
- HOW IT WAS DEVELOPED
-----------------------------------------------------------------------------------------------------------------------

By hand. 



-----------------------------------------------------------------------------------------------------------------------
- HOW TO UPGRADE IT
-----------------------------------------------------------------------------------------------------------------------

No need if it works as everything is self contained.  However, .... if CMSIS is upgraded you can also upgrade iSharIPC.



