
-----------------------------------------------------------------------------------------------------------------------
- iSharPART
-----------------------------------------------------------------------------------------------------------------------


The iSharPART is a shared module to be used in the STM32 micro which offers common access to the partition table.



-----------------------------------------------------------------------------------------------------------------------
- HOW TO INSTALL
-----------------------------------------------------------------------------------------------------------------------

Compile using project ishared-part.
- copy file iSharPART.obj into C:\iEmb\shared\isharpart\lib
- copy file iSharPART.h into C:\iEmb\shared\isharpart\api
- burn flash of the STM32 with iSharPART.hex. Use project ishared-part or a yet-to-be-developed installer
  the ishared-part puts the iSharPART.hex into FLASH partition [8K, 12K) at address flashbase+8K and of size 4KB.  
  CAVEAT: the iSharPART.hex uses dedicated RAM at address ?? for a total of ??.


-----------------------------------------------------------------------------------------------------------------------
- HOW TO USE
-----------------------------------------------------------------------------------------------------------------------

Include the following into your project:
- file with symbols: iSharPART.obj
- api file: iSharPART.h
- tell the linker to exclude: FLASH partition [8K, 12K) and RAM partition ???.
- in use of the iSharPART, inside your code ALWAYS: verify the release version in the iSharPART.h vs what obtained by 
  function sharpart_version_get(&ver).




-----------------------------------------------------------------------------------------------------------------------
- HOW IT WAS DEVELOPED
-----------------------------------------------------------------------------------------------------------------------

By hand. 



-----------------------------------------------------------------------------------------------------------------------
- HOW TO UPGRADE IT
-----------------------------------------------------------------------------------------------------------------------

No need if it works as everything is self contained.  However, .... if CMSIS is upgraded you can also upgrade iSharPART.



