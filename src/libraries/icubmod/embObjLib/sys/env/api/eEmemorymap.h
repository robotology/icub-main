
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EEMEMORYMAP_H_
#define _EEMEMORYMAP_H_


/** @file       eEmemorymap.h
    @brief      This header file defined memory map for the shalibs.
    @author     marco.accame@iit.it
    @date       11/03/2011
**/

// - public #define  --------------------------------------------------------------------------------------------------



// --- general properties
// --- mcbstm32c and ems001 have those common params. ems001 may have more storage size in eeprom

#define EENV_ROMSTART                               (0x08000000)
#define EENV_ROMSIZE                                (256*1024)

#define EENV_RAMSTART                               (0x20000000)
#define EENV_RAMSIZE                                (64*1024)

#define EENV_STGSTART                               (0)
#define EENV_STGSIZE                                (8*1024)

// - moduleinfo for a process or a shalib is shifted 512 byte after the entry point
#define EENV_MODULEINFO_OFFSET                      (512)

// --- memory mapping for shal-system: base, part, info. 
// --- they are on top of flash (for 16k), on top of ram (for 2k), on top of eeprom (for 2k)

#define EENV_MEMMAP_SHALSYSTEM_ROMSIZE              (16*1024)
#define EENV_MEMMAP_SHALSYSTEM_ROMADDR              (EENV_ROMSTART + EENV_ROMSIZE - EENV_MEMMAP_SHALSYSTEM_ROMSIZE)

#define EENV_MEMMAP_SHALSYSTEM_RAMSIZE              (2*1024)
#define EENV_MEMMAP_SHALSYSTEM_RAMADDR              (EENV_RAMSTART +  EENV_RAMSIZE - EENV_MEMMAP_SHALSYSTEM_RAMSIZE)

#define EENV_MEMMAP_SHALSYSTEM_STGSIZE              (2*1024)
#define EENV_MEMMAP_SHALSYSTEM_STGADDR              (EENV_STGSTART +  EENV_STGSIZE - EENV_MEMMAP_SHALSYSTEM_STGSIZE)

// - shalbase: the lowest
// - Execution Region ER_IROM1 (Base: 0x0803d000, Size: 0x00000f80, Max: 0x00001000, ABSOLUTE)
// - Execution Region RW_IRAM1 (Base: 0x2000fc20, Size: 0x00000004, Max: 0x00000020, ABSOLUTE, UNINIT)
#define EENV_MEMMAP_SHALBASE_ROMSIZE                (8*1024)
#define EENV_MEMMAP_SHALBASE_ROMADDR                (EENV_MEMMAP_SHALSYSTEM_ROMADDR + 0)
#define EENV_MEMMAP_SHALBASE_RAMSIZE                (256)
#define EENV_MEMMAP_SHALBASE_RAMADDR                (EENV_MEMMAP_SHALSYSTEM_RAMADDR + 0)
#define EENV_MEMMAP_SHALBASE_STGSIZE                (256)
#define EENV_MEMMAP_SHALBASE_STGADDR                (EENV_MEMMAP_SHALSYSTEM_STGADDR + 0)

#define EENV_MEMMAP_SHALBASE_RAMFOR_ZIDATA          (128)
#define EENV_MEMMAP_SHALBASE_RAMFOR_RWDATA          (EENV_MEMMAP_SHALBASE_RAMSIZE - EENV_MEMMAP_SHALBASE_RAMFOR_ZIDATA)


// - shalpart: the middle
// - Execution Region ER_IROM1 (Base: 0x0803e000, Size: 0x000009ac, Max: 0x00001000, ABSOLUTE)
// - Execution Region RW_IRAM1 (Base: 0x2000ffb4, Size: 0x00000000, Max: 0x00000000, ABSOLUTE, UNINIT) 
#define EENV_MEMMAP_SHALPART_ROMSIZE                (4*1024)
#define EENV_MEMMAP_SHALPART_ROMADDR                (EENV_MEMMAP_SHALBASE_ROMADDR + EENV_MEMMAP_SHALBASE_ROMSIZE)
#define EENV_MEMMAP_SHALPART_RAMSIZE                (1024)
#define EENV_MEMMAP_SHALPART_RAMADDR                (EENV_MEMMAP_SHALBASE_RAMADDR + EENV_MEMMAP_SHALBASE_RAMSIZE)
#define EENV_MEMMAP_SHALPART_STGSIZE                (1024)
#define EENV_MEMMAP_SHALPART_STGADDR                (EENV_MEMMAP_SHALBASE_STGADDR + EENV_MEMMAP_SHALBASE_STGSIZE)

#define EENV_MEMMAP_SHALPART_RAMFOR_ZIDATA          (1024)
#define EENV_MEMMAP_SHALPART_RAMFOR_RWDATA          (EENV_MEMMAP_SHALPART_RAMSIZE - EENV_MEMMAP_SHALPART_RAMFOR_ZIDATA)

// - shalinfo: the top
// - Execution Region ER_IROM1 (Base: 0x0803f000, Size: 0x00000484, Max: 0x00001000, ABSOLUTE)
// - Execution Region RW_IRAM1 (Base: 0x2000ffb4, Size: 0x00000000, Max: 0x00000000, ABSOLUTE, UNINIT)
#define EENV_MEMMAP_SHALINFO_ROMSIZE                (4*1024)
#define EENV_MEMMAP_SHALINFO_ROMADDR                (EENV_MEMMAP_SHALPART_ROMADDR + EENV_MEMMAP_SHALPART_ROMSIZE)
#define EENV_MEMMAP_SHALINFO_RAMSIZE                (768)
#define EENV_MEMMAP_SHALINFO_RAMADDR                (EENV_MEMMAP_SHALPART_RAMADDR + EENV_MEMMAP_SHALPART_RAMSIZE)
#define EENV_MEMMAP_SHALINFO_STGSIZE                (768)
#define EENV_MEMMAP_SHALINFO_STGADDR                (EENV_MEMMAP_SHALPART_STGADDR + EENV_MEMMAP_SHALPART_STGSIZE)

#define EENV_MEMMAP_SHALINFO_RAMFOR_ZIDATA          (768)
#define EENV_MEMMAP_SHALINFO_RAMFOR_RWDATA          (EENV_MEMMAP_SHALINFO_RAMSIZE - EENV_MEMMAP_SHALINFO_RAMFOR_ZIDATA)

// --- memory mapping for shal-abstraction-layer: hal, osal, fsal, ipal
// --- they are just below the shal-system: flash (80k), ram (12k). no eeprom.
// 

#define EENV_MEMMAP_SHALABSLAY_ROMSIZE              (80*1024)
#define EENV_MEMMAP_SHALABSLAY_ROMADDR              (EENV_MEMMAP_SHALSYSTEM_ROMADDR - EENV_MEMMAP_SHALABSLAY_ROMSIZE)

#define EENV_MEMMAP_SHALABSLAY_RAMSIZE              (12*1024)
#define EENV_MEMMAP_SHALABSLAY_RAMADDR              (EENV_MEMMAP_SHALSYSTEM_RAMADDR - EENV_MEMMAP_SHALABSLAY_RAMSIZE)

#define EENV_MEMMAP_SHALABSLAY_STGSIZE              (#error shal abslay does not have eeprom)
#define EENV_MEMMAP_SHALABSLAY_STGADDR              (#error shal abslay does not have eeprom)


// - shal-hal: the lowest (or first from low)
// - Load Region LR_IROM1 (Base: 0x08029000, Size: 0x0000303c, Max: 0x00004000, ABSOLUTE)
// - Execution Region RW_IRAM1 (Base: 0x2000cc08, Size: 0x0000008c, Max: 0x000000f8, ABSOLUTE, UNINIT)
#define EENV_MEMMAP_SHALHAL_ROMSIZE                 (16*1024)
#define EENV_MEMMAP_SHALHAL_ROMADDR                 (EENV_MEMMAP_SHALABSLAY_ROMADDR + 0)
#define EENV_MEMMAP_SHALHAL_RAMSIZE                 (256)
#define EENV_MEMMAP_SHALHAL_RAMADDR                 (EENV_MEMMAP_SHALABSLAY_RAMADDR + 0)

#define EENV_MEMMAP_SHALHAL_RAMFOR_ZIDATA           (8)
#define EENV_MEMMAP_SHALHAL_RAMFOR_RWDATA           (EENV_MEMMAP_SHALHAL_RAMSIZE - EENV_MEMMAP_SHALHAL_RAMFOR_ZIDATA)



// - shal-osal: the second from low

#define EENV_MEMMAP_SHALOSAL_ROMSIZE                 (16*1024)
#define EENV_MEMMAP_SHALOSAL_ROMADDR                 (EENV_MEMMAP_SHALHAL_ROMADDR + EENV_MEMMAP_SHALHAL_ROMSIZE)
#define EENV_MEMMAP_SHALOSAL_RAMSIZE                 (768)
#define EENV_MEMMAP_SHALOSAL_RAMADDR                 (EENV_MEMMAP_SHALHAL_RAMADDR + EENV_MEMMAP_SHALHAL_RAMSIZE)

#define EENV_MEMMAP_SHALOSAL_RAMFOR_ZIDATA           (8)
#define EENV_MEMMAP_SHALOSAL_RAMFOR_RWDATA           (EENV_MEMMAP_SHALOSAL_RAMSIZE - EENV_MEMMAP_SHALOSAL_RAMFOR_ZIDATA)


// - shal-fsal: the third from low

#define EENV_MEMMAP_SHALFSAL_ROMSIZE                 (16*1024)
#define EENV_MEMMAP_SHALFSAL_ROMADDR                 (EENV_MEMMAP_SHALOSAL_ROMADDR + EENV_MEMMAP_SHALOSAL_ROMSIZE)
#define EENV_MEMMAP_SHALFSAL_RAMSIZE                 (256)
#define EENV_MEMMAP_SHALFSAL_RAMADDR                 (EENV_MEMMAP_SHALOSAL_RAMADDR + EENV_MEMMAP_SHALOSAL_RAMSIZE)

#define EENV_MEMMAP_SHALFSAL_RAMFOR_ZIDATA           (8)
#define EENV_MEMMAP_SHALFSAL_RAMFOR_RWDATA           (EENV_MEMMAP_SHALFSAL_RAMSIZE - EENV_MEMMAP_SHALFSAL_RAMFOR_ZIDATA)



// - shal-ipal: the fourth from low (the uppest)

#define EENV_MEMMAP_SHALIPAL_ROMSIZE                 (28*1024)
#define EENV_MEMMAP_SHALIPAL_ROMADDR                 (EENV_MEMMAP_SHALFSAL_ROMADDR + EENV_MEMMAP_SHALFSAL_ROMSIZE)
#define EENV_MEMMAP_SHALIPAL_RAMSIZE                 (9*1024)
#define EENV_MEMMAP_SHALIPAL_RAMADDR                 (EENV_MEMMAP_SHALFSAL_RAMADDR + EENV_MEMMAP_SHALFSAL_RAMSIZE)

#define EENV_MEMMAP_SHALIPAL_RAMFOR_ZIDATA           (8)
#define EENV_MEMMAP_SHALIPAL_RAMFOR_RWDATA           (EENV_MEMMAP_SHALIPAL_RAMSIZE - EENV_MEMMAP_SHALIPAL_RAMFOR_ZIDATA)


// --- memory mapping for system e-processes: eLoader and eUpdater. they are on low of rom and ram. they can share ram

// --- eloader
#define EENV_MEMMAP_ELOADER_ROMSIZE                 (24*1024)
#define EENV_MEMMAP_ELOADER_ROMADDR                 (EENV_ROMSTART + 0)
#define EENV_MEMMAP_ELOADER_RAMSIZE                 (EENV_RAMSIZE-EENV_MEMMAP_SHALSYSTEM_RAMSIZE)
#define EENV_MEMMAP_ELOADER_RAMADDR                 (EENV_RAMSTART) 


// --- eupdater
#ifdef EENV_EUPDATER_FORCE_CODE_OFFSET_TO_ZERO            
   #define EENV_MEMMAP_EUPDATER_ROMADDR             (EENV_ROMSTART + 0)              
#else
#define EENV_MEMMAP_EUPDATER_ROMADDR                (EENV_MEMMAP_ELOADER_ROMADDR + EENV_MEMMAP_ELOADER_ROMSIZE)
#endif
#define EENV_MEMMAP_EUPDATER_ROMSIZE                (72*1024)
#define EENV_MEMMAP_EUPDATER_RAMADDR                (EENV_RAMSTART) 
#define EENV_MEMMAP_EUPDATER_RAMSIZE                (EENV_RAMSIZE-EENV_MEMMAP_SHALSYSTEM_RAMSIZE)



// --- eapplication
#ifdef EENV_EAPPLICATION_FORCE_CODE_OFFSET_TO_ZERO            
   #define EENV_MEMMAP_EAPPLICATION_ROMADDR         (EENV_ROMSTART + 0)              
#else
#define EENV_MEMMAP_EAPPLICATION_ROMADDR            (EENV_MEMMAP_EUPDATER_ROMADDR + EENV_MEMMAP_EUPDATER_ROMSIZE)
#endif
#define EENV_MEMMAP_EAPPLICATION_ROMSIZE            (144*1024)
#define EENV_MEMMAP_EAPPLICATION_RAMADDR            (EENV_RAMSTART) 
#define EENV_MEMMAP_EAPPLICATION_RAMSIZE            (EENV_RAMSIZE-EENV_MEMMAP_SHALSYSTEM_RAMSIZE)
#define EENV_MEMMAP_EAPPLICATION_STGADDR            (EENV_STGSTART) 
#define EENV_MEMMAP_EAPPLICATION_STGSIZE            (EENV_STGSIZE-EENV_MEMMAP_SHALSYSTEM_STGSIZE)



#endif  // include-guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




