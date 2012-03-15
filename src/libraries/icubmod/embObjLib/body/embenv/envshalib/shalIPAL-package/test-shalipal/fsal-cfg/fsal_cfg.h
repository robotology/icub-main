
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _FSAL_CFG_H_
#define _FSAL_CFG_H_

// --------------------------------------------------------------------------------------------------------------------
//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------


// <h> Configuration of FSAL
// <i> It holds configuration of FSAl


// <h> Porting specifics 
// <i> sssssssss

//   <o> Embedded FS         <0=>   IIT-modified FFS-ARM v4.13
//   <i> Only FlashFS v4.13 modified by IIT is now supported.
#define FSAL_GLOB_EMBFSTYPE 0

//   <o> CPU family         <0=>   Cortex M3
//   <i> Only Cortex M3 is now supported.
#define FSAL_GLOB_CPUFAM      0

// </h> Porting specifics


// <h> Global properties 
// <i> sssssssss


//   <o> Max number of openable files <1-16>
//   <i> Define the max number of files which can be opened at teh same time
//   <i> The more files, the more RAM you need
//   <i> Default: 8
#define FSAL_GLOB_FOPENMAX     8

//   <o> Size of defrag buffer <64=> 64 Bytes  <128=> 128 Bytes <256=> 256 Bytes
//   <i> Define the  size of the defrag buffer
//   <i> Default: 256
#define FSAL_GLOB_DEFRAGBUFFERSIZE     256


//   <o> Default Drive  <1=> F:, Embedded FLASH <3=> R:, RAM
//   2= SPI Flash  3= RAM  4= Memory Card
//   <i> This drive is used when a Drive letter is not provided
#define FSAL_GLOB_DEFDRIVE   1



// </h>Global properties


//  <e> Standard IO
//  <i> STDIO on CMx can be on ITM. If on ITM can be displayed by printf window of uVision.

#define FSAL_USE_STDIO 1



//  </e>STDIO



// <h> Drives
// <i> So far only the embedded ram and embedded flash drives

// <e> Embedded RAM
// <i> For volatile storage which uses the heap

#define FSAL_ERAM_ENABLE  0

//   <o> RAM Start address <0=>   Defined by heap
//   <i> Define the start address in RAM 
//   <i> It is defined by the heap
#define FSAL_ERAM_STARTADDRESS 0        

//   <o> Number of sectors <1-16>
//   <i> Define the number of fixed-sized consecutive sectors
//   <i> Default: 4
#define FSAL_ERAM_NUMSECTORS     4

//   <o>Size of each sector in KB <1-16>
//   <i> Define the size in KB of each consecutive sector
//   <i> Default:  2
#define FSAL_ERAM_SIZEKBSECTOR     8


// </e>Embedded RAM     


// <e> Embedded FLASH
// <i> Must be enabled for non volatile storage

#define FSAL_EFLASH_ENABLE  1

    
//   <o> FLASH Start address <0x00000000-0x09000000:0x1000>
//   <i> Define the start address in FLASH (0x08000000 on STM32).
//   <i> Values: +64K->0x08010000, +128K->0x08020000, +196K->0x08030000 
#define FSAL_EFLASH_STARTADDRESS         0x08020000

//   <o> Number of sectors <1-8>
//   <i> Define the number of fixed-sized consecutive sectors
//   <i> Default: 4
#define FSAL_EFLASH_NUMSECTORS     4

//   <o>Size of each sector in KB <2-128:2>
//   <i> Define the size in KB of each consecutive sector. STM32F1x uses multiples of 2K
//   <i> Default:  16
#define FSAL_EFLASH_SIZEKBSECTOR     16


// </e>Embedded FLASH   


// <e> EEPROM (keep unticked: not supported so far)
// <i> Not supported so far

#define FSAL_EEPROM_ENABLE  0

    
//   <o> EEPROM Start address <0x00000000-0x09000000:0x0400>
//   <i> Define the start address in EEPROM .
#define FSAL_EEPROM_STARTADDRESS         0x00000000

//   <o> Number of sectors <1-8>
//   <i> Define the number of fixed-sized consecutive sectors
//   <i> Default: 4
#define FSAL_EEPROM_NUMSECTORS     1

//   <o>Size of each sector in KB <1-255:1>
//   <i> Define the size in KB of each consecutive sector
//   <i> Default:  16
#define FSAL_EEPROM_SIZEKBSECTOR     8

// </e>EEPROM


// <e> SPI FLASH (keep unticked: not supported so far)
// <i> Not supported so far

#define FSAL_SPIFLASH_ENABLE  0

//   <o> SPIFLASH Start address <0x00000000-0x09000000:0x0400>
//   <i> Define the start address in SPIFLASH .
#define FSAL_SPIFLASH_STARTADDRESS         0x00000000

//   <o> Number of sectors <1-8>
//   <i> Define the number of fixed-sized consecutive sectors
//   <i> Default: 4
#define FSAL_SPIFLASH_NUMSECTORS     1

//   <o>Size of each sector in KB <1-255:1>
//   <i> Define the size in KB of each consecutive sector
//   <i> Default:  16
#define FSAL_SPIFLASH_SIZEKBSECTOR     8

// </e>SPIFLASH    


// </h>Drives



// </h>






// --------------------------------------------------------------------------------------------------------------------
//------------- <<< end of configuration section >>> ------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------



#if(FSAL_EEPROM_ENABLE)
    #error EEPROM file system is not supported
#endif

#if(FSAL_SPIFLASH_ENABLE)
    #error SPIFLASH file system is not supported
#endif


#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------


